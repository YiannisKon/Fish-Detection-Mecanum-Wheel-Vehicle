#!/usr/bin/env python3
"""
Fish detection + orientation for a USB webcam on a Raspberry Pi 5 (headless).

Colour-only pipeline (no depth camera): background-subtraction segmentation +
PCA orientation + a head/tail taper heuristic. It serves an annotated MJPEG
stream over HTTP so you can watch it live from another computer's browser, and
prints orientation/heading to the console.

On the Pi:
    python3 fish_detection_pi.py                  # webcam 0, stream on :8000
    python3 fish_detection_pi.py --camera 2       # pick a /dev/videoN index
    python3 fish_detection_pi.py --width 1280 --height 720 --fps 30
    python3 fish_detection_pi.py --seg otsu       # threshold instead of motion
    python3 fish_detection_pi.py --selftest       # validate the math, no camera

Then from your laptop open:   http://fishpi.local:8000

Reported per fish:
    angle     — long-axis angle, 0-180 deg (undirected), image +x = 0, CW
    heading   — 0-360 deg, pointing toward the HEAD (wider end of the body)
    elong     — long/short axis ratio (how elongated the blob is)
    length_px / width_px — size of the fish in pixels (no metric scale w/o depth)
"""
import argparse
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
#  Geometry  (identical, validated math to the RealSense version)
# --------------------------------------------------------------------------- #
def pca_orientation(contour):
    """Region-moment principal-axis analysis of a contour."""
    pts = contour.reshape(-1, 2).astype(np.float64)
    m = cv2.moments(contour)
    if m["m00"] > 1e-6:
        centroid = np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])
        mu20 = m["mu20"] / m["m00"]
        mu02 = m["mu02"] / m["m00"]
        mu11 = m["mu11"] / m["m00"]
        cov = np.array([[mu20, mu11], [mu11, mu02]])
    else:
        centroid = pts.mean(axis=0)
        cov = np.cov((pts - centroid).T)

    centered = pts - centroid
    evals, evecs = np.linalg.eigh(cov)
    order = np.argsort(evals)[::-1]
    evals, evecs = evals[order], evecs[:, order]
    principal, minor = evecs[:, 0], evecs[:, 1]
    angle = np.degrees(np.arctan2(principal[1], principal[0])) % 180.0
    elong = float(np.sqrt(evals[0] / evals[1])) if evals[1] > 1e-9 else 0.0
    return {"centroid": centroid, "principal": principal, "minor": minor,
            "angle": angle, "elongation": elong, "centered": centered}


def head_direction(centered, principal, minor):
    """Wider end of the silhouette is taken as the head. Returns head unit vec."""
    t = centered @ principal
    s = centered @ minor
    if t.size == 0:
        return principal
    span = t.max() - t.min()
    if span <= 1e-6:
        return principal
    hi = s[t > t.max() - 0.25 * span]
    lo = s[t < t.min() + 0.25 * span]
    w_hi = (hi.max() - hi.min()) if hi.size else 0.0
    w_lo = (lo.max() - lo.min()) if lo.size else 0.0
    return principal if w_hi >= w_lo else -principal


def axis_endpoints(centered, principal, centroid):
    t = centered @ principal
    return centroid + principal * t.min(), centroid + principal * t.max()


# --------------------------------------------------------------------------- #
#  Segmentation
# --------------------------------------------------------------------------- #
class Segmenter:
    def __init__(self, mode="motion"):
        self.mode = mode
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=400, varThreshold=32, detectShadows=False)

    def __call__(self, frame, open_k, close_k):
        if self.mode == "otsu":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            _, mask = cv2.threshold(gray, 0, 255,
                                    cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        else:  # motion
            fg = self.bg.apply(frame)
            _, mask = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
        if open_k > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if close_k > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask


def largest_fish_contour(mask, min_area):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    return c if cv2.contourArea(c) >= min_area else None


# --------------------------------------------------------------------------- #
#  Per-frame processing
# --------------------------------------------------------------------------- #
def process_frame(frame, seg, params, state):
    mask = seg(frame, params["open"], params["close"])
    view = frame.copy()

    # Region of interest: blank out a border margin so the tank's edges and the
    # fish's reflection near the frame boundary don't get detected. margin is a
    # fraction (0..0.4) of width/height removed from each side.
    h, w = mask.shape
    margin = params.get("roi", 0.0)
    if margin > 0:
        mx, my = int(w * margin), int(h * margin)
        roi = np.zeros_like(mask)
        roi[my:h - my, mx:w - mx] = 255
        mask = cv2.bitwise_and(mask, roi)
        cv2.rectangle(view, (mx, my), (w - mx, h - my), (140, 140, 140), 1)

    contour = largest_fish_contour(mask, params["min_area"])
    info = None
    if contour is not None:
        pca = pca_orientation(contour)
        head = head_direction(pca["centered"], pca["principal"], pca["minor"])
        sh = state.get("smooth_head")
        sh = head if sh is None else 0.8 * sh + 0.2 * head
        sh = sh / (np.linalg.norm(sh) + 1e-9)
        state["smooth_head"] = sh

        p_lo, p_hi = axis_endpoints(pca["centered"], pca["principal"], pca["centroid"])
        heading = np.degrees(np.arctan2(sh[1], sh[0])) % 360.0
        rect = cv2.minAreaRect(contour)
        (w_px, h_px) = rect[1]
        length_px, width_px = max(w_px, h_px), min(w_px, h_px)
        info = {"angle": pca["angle"], "heading": heading,
                "elongation": pca["elongation"],
                "length_px": length_px, "width_px": width_px}

        cx, cy = pca["centroid"]
        cv2.drawContours(view, [contour], -1, (0, 255, 0), 2)
        box = cv2.boxPoints(rect).astype(int)
        cv2.polylines(view, [box], True, (0, 220, 220), 1)
        cv2.line(view, tuple(p_lo.astype(int)), tuple(p_hi.astype(int)), (255, 180, 0), 2)
        tip = np.array([cx, cy]) + sh * (0.5 * np.linalg.norm(p_hi - p_lo))
        cv2.arrowedLine(view, (int(cx), int(cy)), tuple(tip.astype(int)),
                        (0, 0, 255), 3, tipLength=0.25)
        cv2.circle(view, (int(cx), int(cy)), 4, (255, 255, 255), -1)

        for i, t in enumerate([
                f"angle:   {info['angle']:6.1f} deg",
                f"heading: {info['heading']:6.1f} deg",
                f"elong:   {info['elongation']:5.2f}",
                f"size:    {length_px:4.0f} x {width_px:3.0f} px"]):
            y = 24 + i * 24
            cv2.putText(view, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(view, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 255, 50), 1, cv2.LINE_AA)
    else:
        cv2.putText(view, "no fish (needs motion; tune min_area / --seg otsu)",
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    return view, mask, info


# --------------------------------------------------------------------------- #
#  MJPEG streaming server (stdlib only)
# --------------------------------------------------------------------------- #
class FrameBuffer:
    """Holds the latest annotated JPEG; wakes streaming clients on update."""
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.info = None
        self.seq = 0

    def update(self, bgr, info):
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self.cond:
            self.jpeg = buf.tobytes()
            self.info = info
            self.seq += 1
            self.cond.notify_all()

    def wait(self, last_seq, timeout=5.0):
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            return self.jpeg, self.seq


PAGE = b"""<!doctype html><html><head><meta charset=utf-8>
<title>Fish detection - Pi</title>
<style>body{background:#111;color:#eee;font-family:system-ui;margin:0;text-align:center}
img{max-width:100vw;max-height:90vh;margin-top:1vh;border:1px solid #333}
h1{font-size:1rem;font-weight:500;color:#8f8;margin:.5rem}</style></head>
<body><h1>Fish detection &amp; orientation - live from the Pi</h1>
<img src="/stream"></body></html>"""


def make_handler(buffer):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass  # quiet

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        jpeg, seq = buffer.wait(seq)
                        if jpeg is None:
                            time.sleep(0.05)
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            ("Content-Length: %d\r\n\r\n" % len(jpeg)).encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)
    return Handler


def start_server(port, buffer):
    srv = ThreadingHTTPServer(("0.0.0.0", port), make_handler(buffer))
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv


# --------------------------------------------------------------------------- #
#  Capture + main loop
# --------------------------------------------------------------------------- #
def open_camera(index, width, height, fps):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)  # fallback backend
    # MJPG lets most USB webcams hit higher res/fps than raw YUYV
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def run(args):
    # Release the camera cleanly on `kill` (SIGTERM), not just Ctrl-C, so a
    # restart never leaves the webcam device wedged.
    def _sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    cap = open_camera(args.camera, args.width, args.height, args.fps)
    if not cap.isOpened():
        print(f"[!] Could not open camera index {args.camera}.")
        print("    List cameras with:  ls /dev/video*   or   v4l2-ctl --list-devices")
        return 2

    buffer = FrameBuffer()
    srv = start_server(args.port, buffer)
    print(f"[i] streaming on http://<pi-address>:{args.port}   (e.g. http://fishpi.local:{args.port})")
    print("[i] Ctrl-C to stop")

    seg = Segmenter(args.seg)
    params = {"min_area": args.min_area, "open": 3, "close": 9, "roi": args.roi}
    print(f"[i] ROI margin: {args.roi:.2f} (ignoring frame border)")
    state = {}
    frames = 0
    t0 = time.time()
    last_print = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[!] frame grab failed; retrying...")
                time.sleep(0.1)
                continue
            view, mask, info = process_frame(frame, seg, params, state)
            buffer.update(view, info)
            frames += 1

            now = time.time()
            if now - last_print >= 1.0:
                fps = frames / (now - t0)
                if info:
                    print(f"[fish] angle={info['angle']:6.1f}  heading={info['heading']:6.1f}"
                          f"  elong={info['elongation']:.2f}  size={info['length_px']:.0f}x{info['width_px']:.0f}px"
                          f"  | {fps:4.1f} fps")
                else:
                    print(f"[ .. ] no fish   | {fps:4.1f} fps")
                last_print = now
    except KeyboardInterrupt:
        print("\n[i] stopping")
    finally:
        cap.release()
        srv.shutdown()
    return 0


# --------------------------------------------------------------------------- #
def selftest():
    print("[selftest] PCA orientation on synthetic tapered blobs")
    ok = True
    for true_deg in (0, 20, 45, 70, 90, 120, 160):
        canvas = np.zeros((400, 400), np.uint8)
        cv2.ellipse(canvas, (200, 200), (110, 42), true_deg, 0, 360, 255, -1)
        cnts, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        got = pca_orientation(max(cnts, key=cv2.contourArea))["angle"]
        err = min(abs(got - true_deg), 180 - abs(got - true_deg))
        ok &= err < 2.0
        print(f"  true={true_deg:5.1f}  meas={got:6.1f}  err={err:4.1f}  [{'ok' if err<2 else 'BAD'}]")
    print("[selftest]", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="Webcam fish detection for Raspberry Pi")
    ap.add_argument("--camera", type=int, default=0, help="/dev/videoN index")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--seg", choices=["motion", "otsu"], default="motion")
    ap.add_argument("--min-area", type=int, default=1200, dest="min_area")
    ap.add_argument("--roi", type=float, default=0.12,
                    help="ignore this fraction of the frame border (0=off, e.g. 0.15)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    return selftest() if args.selftest else run(args)


if __name__ == "__main__":
    sys.exit(main())
