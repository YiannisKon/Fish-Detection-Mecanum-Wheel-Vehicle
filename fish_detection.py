#!/usr/bin/env python3
"""
Fish detection + orientation using an Intel RealSense D435i (or any RealSense D400).

What it does
------------
1. Streams aligned COLOR + DEPTH from the RealSense.
2. Segments the fish as the nearest object inside a tunable depth "working
   volume" (this is what a depth camera is *good* at -- no ML weights needed).
3. Fits the fish's principal axis with PCA and reports:
       - orientation angle in the image plane (0-180, undirected)
       - a directed heading (0-360) using a head/tail taper heuristic
       - the 3D position (X, Y, Z metres) of the fish centroid
       - the real-world length of the fish (mm) from the depth of its endpoints
4. Draws an overlay: contour, oriented box, head arrow, and the numbers.

Modes
-----
  python fish_detection.py                 # live RealSense (default)
  python fish_detection.py --bag clip.bag  # playback a recorded .bag (has depth)
  python fish_detection.py --video fish.mp4 # colour-only video (no depth); good for testing
  python fish_detection.py --selftest      # no camera; validates the orientation math

Keys (while a window is focused)
  q / ESC : quit
  m       : toggle the segmentation-mask view
  p       : pause / resume

Tune the depth range and blob size with the trackbars on the "controls" window.
"""

import argparse
import sys
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
    HAVE_RS = True
except Exception:            # pragma: no cover - depends on local build
    rs = None
    HAVE_RS = False


# ----------------------------------------------------------------------------- #
#  Geometry helpers (camera-independent, unit-testable)
# ----------------------------------------------------------------------------- #
def pca_orientation(contour):
    """Principal-axis analysis of a contour.

    Uses the *region* second-order moments (via Green's theorem, i.e.
    cv2.moments of the filled polygon) rather than PCA over boundary pixels --
    the region covariance is far more accurate for the long-axis angle. Returns
    a dict with centroid, principal/minor unit vectors, undirected angle
    (deg, 0-180) and an elongation ratio. Pure/​deterministic so --selftest can
    exercise it.
    """
    pts = contour.reshape(-1, 2).astype(np.float64)
    m = cv2.moments(contour)

    if m["m00"] > 1e-6:
        centroid = np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])
        # normalised central moments == region covariance matrix
        mu20 = m["mu20"] / m["m00"]
        mu02 = m["mu02"] / m["m00"]
        mu11 = m["mu11"] / m["m00"]
        cov = np.array([[mu20, mu11], [mu11, mu02]])
    else:                                    # degenerate/open contour fallback
        centroid = pts.mean(axis=0)
        cov = np.cov((pts - centroid).T)

    centered = pts - centroid

    # eigh returns ascending eigenvalues; largest == long axis.
    evals, evecs = np.linalg.eigh(cov)
    order = np.argsort(evals)[::-1]
    evals = evals[order]
    evecs = evecs[:, order]

    principal = evecs[:, 0]
    minor = evecs[:, 1]

    angle = np.degrees(np.arctan2(principal[1], principal[0])) % 180.0
    elongation = float(np.sqrt(evals[0] / evals[1])) if evals[1] > 1e-9 else 0.0
    return {
        "centroid": centroid,
        "principal": principal,
        "minor": minor,
        "angle": angle,
        "elongation": elongation,
        "centered": centered,
    }


def head_direction(centered, principal, minor):
    """Decide which end of the body is the head.

    A fish is broad at the head/gills and tapers to the tail, so the wider end
    of the silhouette is taken as the head. Returns (head_unit_vector,
    width_head, width_tail). Heuristic -- see README notes.
    """
    t = centered @ principal        # position along the body axis
    s = centered @ minor            # perpendicular offset
    if t.size == 0:
        return principal, 0.0, 0.0

    span = t.max() - t.min()
    if span <= 1e-6:
        return principal, 0.0, 0.0

    hi = s[t > t.max() - 0.25 * span]     # near the +axis tip
    lo = s[t < t.min() + 0.25 * span]     # near the -axis tip
    width_hi = float(hi.max() - hi.min()) if hi.size else 0.0
    width_lo = float(lo.max() - lo.min()) if lo.size else 0.0

    if width_hi >= width_lo:
        return principal, width_hi, width_lo          # head toward +axis
    return -principal, width_lo, width_hi             # head toward -axis


def axis_endpoints(centered, principal, centroid):
    """The two extreme points of the contour along the principal axis (image px)."""
    t = centered @ principal
    p_hi = centroid + principal * t.max()
    p_lo = centroid + principal * t.min()
    return p_lo, p_hi


# ----------------------------------------------------------------------------- #
#  Frame sources
# ----------------------------------------------------------------------------- #
class RealSenseSource:
    """Live camera or .bag playback. Yields aligned colour + depth-in-metres."""

    def __init__(self, width=640, height=480, fps=30, bag=None):
        if not HAVE_RS:
            raise RuntimeError(
                "pyrealsense2 is not importable. Build it first (see README) "
                "or run with --video / --selftest."
            )
        self.pipe = rs.pipeline()
        cfg = rs.config()
        if bag:
            rs.config.enable_device_from_file(cfg, bag, repeat_playback=True)
        else:
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        try:
            self.profile = self.pipe.start(cfg)
        except RuntimeError as e:
            msg = str(e)
            if "power state" in msg or "access" in msg.lower() or "claim" in msg.lower():
                raise RuntimeError(
                    "Could not open the RealSense (\"" + msg + "\").\n"
                    "  On macOS this almost always means the Camera privacy "
                    "permission is not granted to the app running this script.\n"
                    "  Fix: System Settings > Privacy & Security > Camera > enable "
                    "the app (e.g. Terminal), then fully quit & reopen it.\n"
                    "  Also make sure no other app (realsense-viewer, another "
                    "script) is holding the camera."
                ) from e
            raise
        self.align = rs.align(rs.stream.color)          # depth -> colour frame

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()   # z16 units -> metres
        self.intrinsics = None
        self.name = self.profile.get_device().get_info(rs.camera_info.name)

    def read(self):
        frames = self.pipe.wait_for_frames()
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        if self.intrinsics is None:
            self.intrinsics = (
                depth_frame.profile.as_video_stream_profile().intrinsics
            )

        color = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())        # uint16
        depth_m = depth_raw.astype(np.float32) * self.depth_scale  # metres
        return color, depth_m

    def deproject(self, px, py, z):
        """Pixel (px,py) at depth z metres -> (X,Y,Z) metres in camera space."""
        if self.intrinsics is None or z <= 0:
            return None
        return rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(px), float(py)], float(z))

    def close(self):
        try:
            self.pipe.stop()
        except Exception:
            pass


class VideoSource:
    """Colour-only fallback (webcam file / mp4). No depth => motion segmentation."""

    def __init__(self, path):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video: {path}")
        self.bg = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=32, detectShadows=False)
        self.depth_scale = None
        self.intrinsics = None
        self.name = f"video:{path}"

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame, None

    def deproject(self, px, py, z):
        return None

    def close(self):
        self.cap.release()


# ----------------------------------------------------------------------------- #
#  Segmentation
# ----------------------------------------------------------------------------- #
def segment_depth(depth_m, near_m, far_m, open_k, close_k):
    """Foreground = anything inside [near, far] metres. Cleaned with morphology."""
    valid = depth_m > 0
    mask = ((depth_m >= near_m) & (depth_m <= far_m) & valid).astype(np.uint8) * 255
    if open_k > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    if close_k > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def segment_motion(source, frame, open_k, close_k):
    """Colour-only fallback: MOG2 background subtraction."""
    fg = source.bg.apply(frame)
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
    if cv2.contourArea(c) < min_area:
        return None
    return c


# ----------------------------------------------------------------------------- #
#  Drawing
# ----------------------------------------------------------------------------- #
def draw_overlay(img, contour, pca, head_vec, p_lo, p_hi, info):
    cx, cy = pca["centroid"]
    cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)

    # oriented bounding box
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(int)
    cv2.polylines(img, [box], True, (0, 220, 220), 1)

    # principal axis (thin) + head arrow (thick, red)
    cv2.line(img, tuple(p_lo.astype(int)), tuple(p_hi.astype(int)), (255, 180, 0), 2)
    tip = np.array([cx, cy]) + head_vec * (0.5 * np.linalg.norm(p_hi - p_lo))
    cv2.arrowedLine(img, (int(cx), int(cy)), tuple(tip.astype(int)),
                    (0, 0, 255), 3, tipLength=0.25)
    cv2.circle(img, (int(cx), int(cy)), 4, (255, 255, 255), -1)

    lines = [
        f"angle(axis):   {info['angle']:6.1f} deg",
        f"heading(head): {info['heading']:6.1f} deg",
        f"elongation:    {info['elongation']:5.2f}",
    ]
    if info.get("distance_m") is not None:
        lines.append(f"distance:      {info['distance_m']:5.2f} m")
    if info.get("length_mm") is not None:
        lines.append(f"length:        {info['length_mm']:5.0f} mm")

    y = 24
    for t in lines:
        cv2.putText(img, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 255, 50), 1, cv2.LINE_AA)
        y += 26
    return img


# ----------------------------------------------------------------------------- #
#  Trackbar UI
# ----------------------------------------------------------------------------- #
CONTROLS = "controls"


def make_controls(has_depth):
    cv2.namedWindow(CONTROLS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS, 420, 200)
    if has_depth:
        # D435i works from ~0.3 m; default volume 0.3-1.5 m in front of the lens
        cv2.createTrackbar("near_cm", CONTROLS, 30, 500, lambda v: None)
        cv2.createTrackbar("far_cm", CONTROLS, 150, 800, lambda v: None)
    cv2.createTrackbar("min_area", CONTROLS, 1500, 40000, lambda v: None)
    cv2.createTrackbar("open", CONTROLS, 3, 25, lambda v: None)
    cv2.createTrackbar("close", CONTROLS, 9, 25, lambda v: None)


def read_controls(has_depth):
    near = cv2.getTrackbarPos("near_cm", CONTROLS) / 100.0 if has_depth else None
    far = cv2.getTrackbarPos("far_cm", CONTROLS) / 100.0 if has_depth else None
    return {
        "near": near,
        "far": far,
        "min_area": cv2.getTrackbarPos("min_area", CONTROLS),
        "open": cv2.getTrackbarPos("open", CONTROLS),      # 0 => morphology skipped
        "close": cv2.getTrackbarPos("close", CONTROLS),
    }


# ----------------------------------------------------------------------------- #
#  Main loop
# ----------------------------------------------------------------------------- #
def process_frame(color, depth_m, has_depth, source, params, state):
    """Segment -> largest contour -> orientation -> 3D facts -> annotated view.

    Testable in isolation: pass synthetic colour/depth plus a `source` exposing
    .deproject(px,py,z). Returns (annotated_bgr, mask, info_or_None). `state`
    carries the smoothed heading vector across frames (mutated in place).
    """
    if has_depth and depth_m is not None:
        mask = segment_depth(depth_m, params["near"], params["far"],
                             params["open"], params["close"])
    else:
        mask = segment_motion(source, color, params["open"], params["close"])

    contour = largest_fish_contour(mask, params["min_area"])
    view = color.copy()
    if contour is None:
        cv2.putText(view, "no fish (adjust depth range / min_area)",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        return view, mask, None

    pca = pca_orientation(contour)
    head_vec, _, _ = head_direction(pca["centered"], pca["principal"], pca["minor"])

    # steady the heading with an exponential moving average
    sh = state.get("smooth_head")
    sh = head_vec if sh is None else 0.8 * sh + 0.2 * head_vec
    sh = sh / (np.linalg.norm(sh) + 1e-9)
    state["smooth_head"] = sh

    p_lo, p_hi = axis_endpoints(pca["centered"], pca["principal"], pca["centroid"])
    heading = np.degrees(np.arctan2(sh[1], sh[0])) % 360.0
    info = {"angle": pca["angle"], "heading": heading,
            "elongation": pca["elongation"],
            "distance_m": None, "length_mm": None}

    # depth-derived 3D facts (distance + real length).
    # Use the MEDIAN depth over the fish silhouette, not single pixels: the
    # centroid may fall on a hole and the axis tips sit on the silhouette edge
    # where depth bleeds onto the background. Length is then the tip-to-tip
    # lateral span deprojected at the fish's distance (assumes the fish is
    # roughly fronto-parallel; strong depth-tilt would need per-segment depth).
    if has_depth and depth_m is not None:
        h, w = depth_m.shape
        fish_mask = np.zeros((h, w), np.uint8)
        cv2.drawContours(fish_mask, [contour], -1, 255, -1)
        zsel = depth_m[(fish_mask > 0) & (depth_m > 0)]
        if zsel.size:
            z_fish = float(np.median(zsel))
            info["distance_m"] = z_fish
            a = source.deproject(p_lo[0], p_lo[1], z_fish)
            b = source.deproject(p_hi[0], p_hi[1], z_fish)
            if a and b:
                info["length_mm"] = float(np.linalg.norm(np.array(a) - np.array(b)) * 1000.0)

    draw_overlay(view, contour, pca, sh, p_lo, p_hi, info)
    return view, mask, info


def run(source, has_depth):
    make_controls(has_depth)
    show_mask = False
    paused = False
    state = {}
    last = None

    print(f"[i] source: {source.name}")
    print("[i] keys: q/ESC quit | m mask | p pause")

    while True:
        if not paused:
            got = source.read()
            if got is None:
                print("[i] end of stream")
                break
            last = got
        color, depth_m = last

        params = read_controls(has_depth)
        view, mask, info = process_frame(color, depth_m, has_depth, source, params, state)

        cv2.imshow("fish detection", view)
        if show_mask:
            cv2.imshow("mask", mask)
        elif cv2.getWindowProperty("mask", cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow("mask")

        # optional: colourised depth for context (normalise over valid pixels only)
        if has_depth and depth_m is not None:
            valid = depth_m[depth_m > 0]
            p95 = float(np.percentile(valid, 95)) if valid.size else 4.0
            dcm = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_m, alpha=255.0 / max(0.5, p95)),
                cv2.COLORMAP_JET)
            cv2.imshow("depth", dcm)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("m"):
            show_mask = not show_mask
        if key == ord("p"):
            paused = not paused

    source.close()
    cv2.destroyAllWindows()


# ----------------------------------------------------------------------------- #
#  Self-test: no camera needed, validates the orientation math
# ----------------------------------------------------------------------------- #
def selftest():
    print("[selftest] validating PCA orientation on synthetic fish blobs")
    ok = True
    for true_deg in (0, 20, 45, 70, 90, 120, 160):
        canvas = np.zeros((400, 400), np.uint8)
        # a tapered ellipse (wide 'head', narrow 'tail') rotated by true_deg
        cv2.ellipse(canvas, (200, 200), (110, 42), true_deg, 0, 360, 255, -1)
        cv2.circle(canvas, (200, 200), 3, 0, -1)  # noise
        cnts, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pca = pca_orientation(max(cnts, key=cv2.contourArea))
        got = pca["angle"]
        err = min(abs(got - true_deg), 180 - abs(got - true_deg))
        status = "ok " if err < 2.0 else "BAD"
        if err >= 2.0:
            ok = False
        print(f"  true={true_deg:5.1f}  measured={got:6.1f}  err={err:4.1f}  [{status}]")
    print("[selftest]", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ----------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="RealSense fish detection + orientation")
    ap.add_argument("--bag", help="play back a recorded .bag file (has depth)")
    ap.add_argument("--video", help="colour-only video/webcam file (no depth)")
    ap.add_argument("--selftest", action="store_true", help="validate math, no camera")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.video:
        source = VideoSource(args.video)
        has_depth = False
    else:
        source = RealSenseSource(args.width, args.height, args.fps, bag=args.bag)
        has_depth = True

    try:
        run(source, has_depth)
    except KeyboardInterrupt:
        source.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
