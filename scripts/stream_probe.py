"""Headless probe: can we actually START the RealSense stream and pull frames?
Saves a colour PNG + colourised depth PNG so we can eyeball what the camera sees.
"""
import sys, time
import numpy as np
import cv2
import pyrealsense2 as rs

OUT = "."

ctx = rs.context()
devs = ctx.query_devices()
print("devices:", len(devs))
for d in devs:
    try:
        print("  name:", d.get_info(rs.camera_info.name),
              "| fw:", d.get_info(rs.camera_info.firmware_version),
              "| sn:", d.get_info(rs.camera_info.serial_number))
    except Exception as e:
        print("  (could not read device info:", e, ")")

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("starting pipeline...")
t0 = time.time()
try:
    profile = pipe.start(cfg)
except Exception as e:
    print("STREAM START FAILED:", repr(e))
    sys.exit(2)
print(f"pipeline started in {time.time()-t0:.2f}s")

depth_sensor = profile.get_device().first_depth_sensor()
scale = depth_sensor.get_depth_scale()
align = rs.align(rs.stream.color)

color = depth = None
got = 0
for i in range(60):
    try:
        frames = pipe.wait_for_frames(timeout_ms=5000)
    except Exception as e:
        print("wait_for_frames error:", repr(e))
        break
    frames = align.process(frames)
    c = frames.get_color_frame()
    d = frames.get_depth_frame()
    if not c or not d:
        continue
    got += 1
    color = np.asanyarray(c.get_data())
    depth = np.asanyarray(d.get_data()).astype(np.float32) * scale
    if got >= 30:      # let auto-exposure settle
        break

pipe.stop()
print("frames captured:", got)
if color is not None:
    h, w = depth.shape
    cz = float(depth[h // 2, w // 2])
    valid = depth[(depth > 0)]
    print(f"color shape: {color.shape}")
    print(f"center distance: {cz:.3f} m")
    if valid.size:
        print(f"depth valid px: {valid.size} | min {valid.min():.2f} m  max {valid.max():.2f} m  median {np.median(valid):.2f} m")
    cv2.imwrite(f"{OUT}/probe_color.png", color)
    dcm = cv2.applyColorMap(
        cv2.convertScaleAbs(depth, alpha=255.0 / max(0.5, np.percentile(depth[depth>0] if valid.size else depth, 95))),
        cv2.COLORMAP_JET)
    cv2.imwrite(f"{OUT}/probe_depth.png", dcm)
    print("saved probe_color.png and probe_depth.png")
    print("STREAM OK")
else:
    print("NO FRAMES")
