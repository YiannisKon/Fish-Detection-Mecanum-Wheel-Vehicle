#!/usr/bin/env python3
"""
Fish pilot: a goldfish drives a mecanum-wheel vehicle.

An overhead webcam watches the tank mounted on the car. The fish is detected
by color, its heading is estimated from body shape (the head end is wider than
the tail), and the resulting drive command is streamed to a Pico that runs the
motors.

Control modes:
  --mode heading (default)
      Direction: wherever the fish points, the car goes.
      Speed (--speed accel, default): builds while the fish actively swims
      along its facing and decays while it rests, like a throttle it has to
      keep earning. --speed radial instead maps speed to the fish's distance
      from the tank center.
  --mode position
      The tank is a joystick: offset from center sets both direction and
      speed, with a rest spot in the middle.

Run on the Pi (Pico on /dev/ttyACM0, webcam auto-detected):
    python3 fish_drive.py --dry-run        # HUD only, motors off
    python3 fish_drive.py                  # fish drives for real

HUD stream: http://<pi>:8000 (zone ring, fish outline, heading + command arrows)

Safety:
  * the car stays parked until the fish first enters the center zone
  * SPACE in the terminal is an emergency stop; WASD/QEZC/JL override manually
  * fish lost or stream stalled -> command decays to zero
  * the Pico firmware adds gentle accel/decel ramps and a 400 ms failsafe stop
"""
import argparse
import glob
import math
import select
import signal
import sys
import termios
import time
import tty

import cv2
import numpy as np

import fish_detection_pi as fd


# --------------------------------------------------------------------------- #
#  Fish position  ->  car velocity   (pure & unit-testable)
# --------------------------------------------------------------------------- #
def cam_to_car(ix, iy, rot=0, mirror=False):
    """Map an image-frame vector (x right, y down) to car frame (vx fwd+, vy left+).

    Default mounting (rot=0): image TOP = car FRONT. --rot 90/180/270 rotates
    for other camera mountings; --mirror flips left/right first.
    """
    if mirror:
        ix = -ix
    vx, vy = -iy, -ix                        # image up = +vx, image right = -vy
    a = math.radians(rot)
    return (vx * math.cos(a) - vy * math.sin(a),
            vx * math.sin(a) + vy * math.cos(a))


def position_to_velocity(cx, cy, w, h, deadzone, max_speed, rot=0, mirror=False,
                         full_at=0.65):
    """Joystick mapping: normalized offset from frame center -> (vx, vy).

    The steady-state speed matcher: a fish swimming at constant speed settles
    at the offset where car speed == fish speed (treadmill equilibrium).

    Throttle ramps LINEARLY from the deadzone edge and hits FULL speed at
    r == full_at (not the frame edge, which the ROI crop makes unreachable) —
    so the whole speed range lives inside the area the fish can actually use.
    """
    nx = (cx - w / 2.0) / (w / 2.0)          # -1 (left)  .. +1 (right)
    ny = (cy - h / 2.0) / (h / 2.0)          # -1 (top)   .. +1 (bottom)
    r = math.hypot(nx, ny)
    if r < 1e-6:
        return 0.0, 0.0, 0.0
    m = (r - deadzone) / max(1e-6, full_at - deadzone)
    if m <= 0.0:
        return 0.0, 0.0, r
    speed = min(1.0, m)                      # linear & reactive
    ux, uy = nx / r, ny / r
    vx, vy = cam_to_car(ux * speed, uy * speed, rot, mirror)
    return max_speed * vx, max_speed * vy, r


def velocity_feedforward(v_px, w, h, kv, rot=0, mirror=False):
    """Fish swim velocity (px/s, image frame) -> instant car command term.

    Normalized isotropically (same px/s -> same command in any direction):
    a fish swimming half-frame-width per second == 1.0, scaled by kv. This is
    the 'speed matching' term: a burst of swimming moves the car immediately,
    without waiting for the fish to drift off-center.
    """
    half = max(w, h) / 2.0
    nvx = v_px[0] / half
    nvy = v_px[1] / half
    vx, vy = cam_to_car(nvx, nvy, rot, mirror)
    return kv * vx, kv * vy


def heading_command(head_img, vel_px, w, h, cruise, dt, accel, decay,
                    effort_min, effort_ref, max_speed, rot=0, mirror=False):
    """ACCELERATION speed for heading mode: the car drives WHERE THE FISH
    FACES; speed is an integrator gated by swim effort:
        * fish actively swims along its facing -> car accelerates
        * fish rests / drifts                  -> speed decays
    Returns (vx, vy, new_cruise, effort)."""
    half = max(w, h) / 2.0
    nvx, nvy = vel_px[0] / half, vel_px[1] / half
    effort = max(0.0, nvx * head_img[0] + nvy * head_img[1])
    if effort >= effort_min:
        cruise += accel * dt * min(1.0, effort / effort_ref)
    else:
        cruise -= decay * dt
    cruise = max(0.0, min(max_speed, cruise))
    hx, hy = cam_to_car(head_img[0], head_img[1], rot, mirror)
    n = math.hypot(hx, hy)
    if n < 1e-9:
        return 0.0, 0.0, cruise, effort
    return cruise * hx / n, cruise * hy / n, cruise, effort


def heading_radial_command(head_img, rnorm, rest, full_at, max_speed,
                           rot=0, mirror=False):
    """HEADING mode: the car drives WHEREVER THE FISH POINTS.

    Direction = the head vector (wide end of the body), no matter where the
    fish sits in the tank. Speed = the fish's DISTANCE FROM TANK CENTER:
    parked inside the rest spot, ramping linearly to full speed at full_at.
    No integration — instant, predictable mapping. Pure -> unit-testable.
    """
    m = (rnorm - rest) / max(1e-6, full_at - rest)
    speed = max_speed * min(1.0, max(0.0, m))
    hx, hy = cam_to_car(head_img[0], head_img[1], rot, mirror)
    n = math.hypot(hx, hy)
    if n < 1e-9 or speed <= 0.0:
        return 0.0, 0.0
    return speed * hx / n, speed * hy / n


class ColorSegmenter:
    """STRICT ORANGE gate for the fish — two independent color tests ANDed:

    1. HSV: hue in the orange band with a real saturation floor (washed-out
       brown tank floor fails this).
    2. LAB a-channel >= min_a: actual redness. Gray/brown haze sits ~128-135;
       an orange fish reads clearly higher. This kills the "whole tank floor
       becomes one giant blob" failure regardless of lighting.
    """

    def __init__(self, lo, hi, min_a=140):
        self.lo = np.array(lo, np.uint8)
        self.hi = np.array(hi, np.uint8)
        self.min_a = min_a

    def __call__(self, frame, open_k, close_k):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lo, self.hi)
        lab_a = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)[:, :, 1]
        mask = cv2.bitwise_and(mask, (lab_a >= self.min_a).astype(np.uint8) * 255)
        if open_k > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if close_k > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask


def pick_fish_contour(mask, min_area, max_area):
    """Largest contour within a fish-plausible AREA WINDOW. A blob bigger than
    max_area (e.g. the tank floor) cannot be the fish and is never picked."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_a = None, 0.0
    for c in cnts:
        a = cv2.contourArea(c)
        if min_area <= a <= max_area and a > best_a:
            best, best_a = c, a
    return best


def parse_hsv(s):
    parts = [int(x) for x in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("HSV must be 'H,S,V' (e.g. '5,70,50')")
    return parts


# --------------------------------------------------------------------------- #
def find_serial():
    for pat in ("/dev/ttyACM*", "/dev/cu.usbmodem*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def open_camera_auto(spec, width, height, fps):
    """Open the webcam without index roulette.

    USB re-enumeration shuffles /dev/videoN between boots, so 'auto' (default)
    prefers the stable /dev/v4l/by-id/*video-index0 symlink, then falls back to
    scanning indices 0-3, verifying each with a real frame read. Pass a number
    to force a specific index.
    """
    if spec == "auto":
        candidates = sorted(glob.glob("/dev/v4l/by-id/*video-index0")) + [0, 1, 2, 3]
    else:
        candidates = [int(spec)]
    for c in candidates:
        cap = cv2.VideoCapture(c, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        ok, _ = cap.read()
        if ok:
            print(f"[i] camera: {c}")
            return cap
        cap.release()
    return None


def draw_hud(view, w, h, deadzone, fish_px, vx, vy, speed_norm, mode, dry, rest=0.0):
    cx0, cy0 = w // 2, h // 2
    # Zone ring: outline only (elliptical because offsets normalize per axis).
    ax, ay = int(deadzone * w / 2), int(deadzone * h / 2)
    cv2.ellipse(view, (cx0, cy0), (ax, ay), 0, 0, 360, (255, 255, 80), 2)
    if rest > 0:  # faint inner rest spot: truly parked only inside this
        cv2.ellipse(view, (cx0, cy0), (int(rest * w / 2), int(rest * h / 2)),
                    0, 0, 360, (140, 140, 90), 1)
    cv2.drawMarker(view, (cx0, cy0), (255, 255, 80), cv2.MARKER_CROSS, 14, 1)
    if fish_px is not None:
        fx, fy = int(fish_px[0]), int(fish_px[1])
        cv2.line(view, (cx0, cy0), (fx, fy), (60, 200, 255), 1)
        cv2.circle(view, (fx, fy), 6, (60, 200, 255), 2)
    # commanded velocity arrow (car frame: vx fwd = up, vy left = image left)
    ax, ay = int(cx0 - vy * 120), int(cy0 - vx * 120)
    cv2.arrowedLine(view, (cx0, cy0), (ax, ay), (0, 0, 255), 3, tipLength=0.3)
    tag = f"{mode}{' DRY-RUN' if dry else ''}  vx={vx:+.2f} vy={vy:+.2f}  |cmd|={speed_norm:.2f}"
    cv2.putText(view, tag, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(view, tag, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 255, 255), 1, cv2.LINE_AA)


def draw_state(view, w, text, color):
    """Big status banner top-center (ARMED / WAITING / E-STOP)."""
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    x = max(8, (w - tw) // 2)
    cv2.putText(view, text, (x, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(view, text, (x, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description="Fish drives the mecanum car")
    ap.add_argument("--camera", default="auto",
                    help="'auto' (default) finds the webcam by stable USB id; "
                         "or a /dev/videoN index number")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--port", type=int, default=8000, help="HUD stream port")
    ap.add_argument("--serial", default=None, help="Pico port (auto: ttyACM*)")
    ap.add_argument("--seg", choices=["color", "motion", "otsu"], default="color",
                    help="color = orange-fish HSV gate (default; ignores shadows)")
    ap.add_argument("--hsv-lo", type=parse_hsv, default=[0, 50, 32], dest="hsv_lo",
                    help="orange lower bound H,S,V")
    ap.add_argument("--hsv-hi", type=parse_hsv, default=[28, 255, 255], dest="hsv_hi",
                    help="orange upper bound H,S,V")
    ap.add_argument("--min-a", type=int, default=139, dest="min_a",
                    help="LAB redness floor (measured: fish p75=143, "
                         "background p99=138)")
    ap.add_argument("--min-area", type=int, default=250, dest="min_area",
                    help="min blob px^2 (the real fish measures ~800)")
    ap.add_argument("--max-area", type=int, default=6000, dest="max_area",
                    help="max blob px^2 — bigger than this cannot be the fish")
    ap.add_argument("--roi", type=float, default=0.04,
                    help="ignored border fraction (small: color mode doesn't "
                         "need edge cropping, and the fish swims near walls)")
    ap.add_argument("--deadzone", type=float, default=0.42,
                    help="the ZONE ring: arming boundary + visual reference")
    ap.add_argument("--rest", type=float, default=0.12,
                    help="inner rest radius: truly parked only within this")
    ap.add_argument("--full-at", type=float, default=0.52, dest="full_at",
                    help="offset radius where throttle reaches 100%%; with "
                         "defaults the nose hits ~75%% speed AT the zone ring")
    ap.add_argument("--max-speed", type=float, default=0.35, dest="max_speed",
                    help="speed cap (0.35 default: keeps tank water aboard)")
    ap.add_argument("--rot", type=int, default=0, choices=[0, 90, 180, 270],
                    help="camera mounting rotation (CCW deg)")
    ap.add_argument("--mirror", action="store_true", help="flip left/right")
    ap.add_argument("--kv", type=float, default=0.3,
                    help="speed-matching gain: how much the fish's own swim "
                         "velocity drives the car instantly (0 = position only)")
    ap.add_argument("--mode", choices=["heading", "position"], default="heading",
                    help="heading = drive WHERE THE FISH POINTS; "
                         "position = tank-as-joystick")
    ap.add_argument("--speed", choices=["accel", "radial"], default="accel",
                    help="heading-mode speed: accel = integrates while the "
                         "fish swims / decays at rest; radial = distance from center")
    ap.add_argument("--accel", type=float, default=0.30,
                    help="accel per second of active swimming")
    ap.add_argument("--decay", type=float, default=0.28,
                    help="speed decay per second while resting")
    ap.add_argument("--effort-min", type=float, default=0.04, dest="effort_min",
                    help="swim speed that counts as 'swimming'")
    ap.add_argument("--effort-ref", type=float, default=0.40, dest="effort_ref",
                    help="swim speed that earns full acceleration")
    ap.add_argument("--manual-speed", type=float, default=0.30, dest="manual_speed",
                    help="speed of the WASD manual override")
    ap.add_argument("--manual-turn", type=float, default=0.42, dest="manual_turn",
                    help="Q/E rotation speed (higher than translation: spinning "
                         "the mecanum base needs more torque)")
    ap.add_argument("--dry-run", action="store_true", help="HUD only, no motor commands")
    ap.add_argument("--send-hz", type=float, default=20.0)
    args = ap.parse_args()

    def _sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    # --- motors ------------------------------------------------------------- #
    ser = None
    if not args.dry_run:
        import serial as pyserial
        port = args.serial or find_serial()
        if not port:
            print("[!] no Pico serial port; use --dry-run to test without motors")
            return 2
        ser = pyserial.Serial(port, 115200, timeout=0)
        print(f"[i] motors: {port}")
    else:
        print("[i] DRY RUN — no motor commands will be sent")

    # --- camera + stream ---------------------------------------------------- #
    cap = open_camera_auto(args.camera, args.width, args.height, args.fps)
    if cap is None:
        print(f"[!] cannot open any camera (tried: {args.camera}). Is the webcam plugged in?")
        return 2
    buffer = fd.FrameBuffer()
    srv = fd.start_server(args.port, buffer)
    print(f"[i] HUD stream: http://<pi>:{args.port}   kv={args.kv}")

    if args.seg == "color":
        seg = ColorSegmenter(args.hsv_lo, args.hsv_hi, args.min_a)
        print(f"[i] segmentation: ORANGE color gate HSV {args.hsv_lo}..{args.hsv_hi}")
    else:
        seg = fd.Segmenter(args.seg)

    # keyboard (over SSH): SPACE = e-stop, hold W/S/A/D/Q/E = manual override
    # (works even while the fish drives — release and the fish resumes), x = quit.
    kb_old = None
    if sys.stdin.isatty():
        kb_old = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        print("[i] keys: SPACE=e-stop | WASD move, QEZC corners, JL rotate | x=quit")

    HOLD_S = 0.45                        # a held key repeats within this window
    manual_seen = {}                     # key -> last auto-repeat time
    cruise = 0.0                         # integrated speed (accel mode)
    smooth = np.zeros(2)                 # EMA of command (kills detection jitter)
    vel_ema = np.zeros(2)                # fish swim velocity EMA
    head_ema = None                      # fish heading EMA (head = wide end)
    last_c = None
    last_t = None
    armed = False                        # no commands until fish first enters zone
    send_period = 1.0 / args.send_hz
    last_send = 0.0
    last_log = 0.0
    print(f"[i] mode: {args.mode}")
    print("[i] WAITING: car is parked until the fish enters the center zone")

    try:
        while True:
            # --- keyboard (e-stop / manual override / quit) ------------------ #
            if kb_old is not None:
                while select.select([sys.stdin], [], [], 0)[0]:
                    k = sys.stdin.read(1).lower()
                    if k == " ":
                        armed = False
                        cruise = 0.0
                        smooth[:] = 0
                        manual_seen.clear()
                        if ser:
                            ser.write(b"S\n")
                        print("\n[E-STOP] motors stopped — fish must re-enter the zone to resume")
                    elif k in "wsadqezcjl":
                        manual_seen[k] = time.time()
                    elif k == "x":
                        raise KeyboardInterrupt

            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            h, w = frame.shape[:2]
            mask = seg(frame, 3, 9)
            # ROI: ignore border (tank edges / reflections)
            if args.roi > 0:
                mx, my = int(w * args.roi), int(h * args.roi)
                roi = np.zeros_like(mask)
                roi[my:h - my, mx:w - mx] = 255
                mask = cv2.bitwise_and(mask, roi)

            contour = pick_fish_contour(mask, args.min_area, args.max_area)
            view = frame.copy()
            vx = vy = 0.0
            rnorm = 0.0
            fish_px = None

            if contour is not None:
                cv2.drawContours(view, [contour], -1, (0, 255, 0), 2)
                pca = fd.pca_orientation(contour)
                cx, cy = pca["centroid"]
                fish_px = (cx, cy)
                now = time.time()

                # fish heading: wide end = head, narrow end = tail (taper)
                head = fd.head_direction(pca["centered"], pca["principal"], pca["minor"])
                head_ema = head if head_ema is None else 0.75 * head_ema + 0.25 * head
                head_ema = head_ema / (np.linalg.norm(head_ema) + 1e-9)

                # CONTROL POINT = the fish's HEAD TIP (nose), not the body
                # center: nosing the zone edge is enough to start driving.
                p_lo, p_hi = fd.axis_endpoints(pca["centered"], pca["principal"],
                                               pca["centroid"])
                tip = p_hi if np.dot(p_hi - pca["centroid"], head_ema) > 0 else p_lo
                cx, cy = float(tip[0]), float(tip[1])
                fish_px = (cx, cy)       # HUD dot marks the nose (control point)

                # fish swim velocity relative to the tank (EMA-filtered, px/s).
                # Tracked on the BODY CENTER, not the nose — a turning fish
                # sweeps its nose sideways, which would fake speed signals.
                bc = np.asarray(pca["centroid"], dtype=float)
                frame_dt = 1.0 / 30.0
                if last_c is not None and last_t and now > last_t:
                    frame_dt = min(0.2, now - last_t)
                    v_px = (bc - last_c) / (now - last_t)
                    vel_ema = 0.7 * vel_ema + 0.3 * v_px
                last_c = bc
                last_t = now

                # nose offset radius (for arming + HUD)
                rnorm = math.hypot((cx - w / 2.0) / (w / 2.0),
                                   (cy - h / 2.0) / (h / 2.0))

                # ARMING: first zone entry enables driving (so the car can't
                # bolt at startup with the fish already off-center)
                if not armed and rnorm < args.deadzone:
                    armed = True
                    print("\n[ARMED] fish entered the zone — driving enabled")

                if args.mode == "heading":
                    # direction = WHERE THE FISH POINTS (always)
                    if args.speed == "radial":
                        vx, vy = heading_radial_command(
                            head_ema, rnorm, args.rest, args.full_at,
                            args.max_speed, args.rot, args.mirror)
                    else:
                        # acceleration: speed builds while the fish swims,
                        # decays while it rests
                        vx, vy, cruise, _effort = heading_command(
                            head_ema, vel_ema, w, h, cruise, frame_dt,
                            args.accel, args.decay, args.effort_min,
                            args.effort_ref, args.max_speed,
                            args.rot, args.mirror)
                else:
                    # position joystick + speed-matching feedforward
                    vx, vy, _ = position_to_velocity(
                        cx, cy, w, h, args.rest, args.max_speed,
                        args.rot, args.mirror, args.full_at)
                    if args.kv > 0 and rnorm >= args.rest:
                        fvx, fvy = velocity_feedforward(
                            vel_ema, w, h, args.kv, args.rot, args.mirror)
                        vx += fvx
                        vy += fvy
                    s = math.hypot(vx, vy)
                    if s > args.max_speed:
                        vx, vy = vx / s * args.max_speed, vy / s * args.max_speed
            else:
                last_c = None
                last_t = None
                vel_ema = np.zeros(2)
                head_ema = None
                cruise = max(0.0, cruise - 0.02)     # fish lost: bleed off

            if not armed:                # parked until first zone entry / re-arm
                vx = vy = 0.0
                cruise = 0.0
                smooth[:] = 0

            # MANUAL OVERRIDE from the laptop: any held drive key takes the
            # wheel instantly (even when not armed — wall rescue); release for
            # HOLD_S and the fish resumes command.
            def _held(key):
                return time.time() - manual_seen.get(key, 0) < HOLD_S
            manual = any(_held(k) for k in "wsadqezcjl")
            wz = 0.0
            if manual:
                ms = args.manual_speed
                # corners are single keys (terminals only auto-repeat the last
                # held key, so W+D combos die — q/e/z/c are reliable corners)
                vx = ms * (_held("w") + _held("q") + _held("e")
                           - _held("s") - _held("z") - _held("c"))
                vy = ms * (_held("a") + _held("q") + _held("z")
                           - _held("d") - _held("e") - _held("c"))
                wz = args.manual_turn * (_held("j") - _held("l"))
                mag = math.hypot(vx, vy)
                if mag > ms:             # corners capped at manual speed too
                    vx, vy = vx / mag * ms, vy / mag * ms
                cruise = 0.0             # fish re-earns speed after an override

            # smooth the command a touch (Pico slew does the heavy lifting);
            # manual override skips smoothing for instant response
            if manual:
                smooth[:] = (vx, vy)
            else:
                smooth = 0.7 * smooth + 0.3 * np.array([vx, vy])
            svx, svy = float(smooth[0]), float(smooth[1])

            now = time.time()
            if ser and now - last_send >= send_period:
                ser.write(f"V {svx:.2f} {svy:.2f} {wz:.2f}\n".encode())
                ser.read(200)            # drain any replies
                last_send = now

            mode_tag = (f"heading/{args.speed} cruise={cruise:.2f}"
                        if args.mode == "heading" else f"pos+kv{args.kv:g}")
            draw_hud(view, w, h, args.deadzone, fish_px, svx, svy, rnorm,
                     mode_tag, args.dry_run, args.rest)
            # fish heading arrow (magenta): head = wide end of the body
            if fish_px is not None and head_ema is not None:
                fx, fy = int(fish_px[0]), int(fish_px[1])
                tip = (int(fx + head_ema[0] * 55), int(fy + head_ema[1] * 55))
                cv2.arrowedLine(view, (fx, fy), tip, (255, 0, 255), 3, tipLength=0.35)
            if manual:
                draw_state(view, w, "MANUAL OVERRIDE", (0, 170, 255))
            elif armed:
                draw_state(view, w, "ARMED - fish is driving", (80, 255, 80))
            else:
                draw_state(view, w, "PARKED - waiting for fish in zone", (80, 200, 255))
            buffer.update(view, None)

            if now - last_log >= 1.0:
                state = ("FISH" if fish_px else "----") + ("/ARM" if armed else "/off")
                extra = f" cruise={cruise:.2f}" if args.mode == "heading" else ""
                print(f"[{state}] vx={svx:+.2f} vy={svy:+.2f}{extra}")
                last_log = now
    except KeyboardInterrupt:
        print("\n[i] stopping")
    finally:
        if kb_old is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, kb_old)
        if ser:
            try:
                ser.write(b"S\n")
                time.sleep(0.1)
                ser.close()
            except Exception:
                pass
        cap.release()
        srv.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
