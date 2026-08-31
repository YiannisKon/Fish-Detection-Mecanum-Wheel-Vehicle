#!/bin/zsh
# RealSense D435i health check via ioreg (reliable on Apple Silicon; system_profiler
# SPUSBDataType returns empty on some Macs). Reports USB link speed + interface state.
NAME="Intel(R) RealSense(TM) Depth Camera 435i"
echo "============================================================"
echo " RealSense D435i connection check"
echo "============================================================"
echo ""

TREE=$(ioreg -p IOUSB -w 0 -l -r -n "$NAME" 2>/dev/null)
if [ -z "$TREE" ]; then
  TREE=$(ioreg -p IOUSB -w 0 -l 2>/dev/null | grep -A40 -i realsense)
fi

echo "1) USB link (no camera permission needed):"
if [ -z "$TREE" ]; then
  echo "   ✗ D435i not found on the USB bus. Replug it."
else
  SPEED=$(echo "$TREE" | grep -m1 "Device Speed" | grep -oE '[0-9]+$')
  NIF=$(echo "$TREE" | grep -c "IOUSBHostInterface")
  case "$SPEED" in
    3|4) echo "   ✓ Device Speed = $SPEED  → USB 3 SuperSpeed (good)";;
    2)   echo "   ✗ Device Speed = 2  → USB 2.0 (480 Mb/s). The D435i needs USB 3.";;
    *)   echo "   ? Device Speed = '$SPEED' (couldn't parse)";;
  esac
  echo "   USB interfaces enumerated: $NIF   (a healthy D435i shows several; 0 = bad link / hung camera)"
  if [ "$SPEED" = "2" ] || [ "$NIF" = "0" ]; then
    echo ""
    echo "   >>> FIX: unplug the camera, wait 5 s, and reconnect it with the"
    echo "       USB-C cable that CAME WITH the D435i, plugged DIRECTLY into a"
    echo "       Mac USB-C / Thunderbolt port — no hub, no dock, no charging cable."
    echo "       Then re-run this check. You want Speed = 3 and interfaces > 0."
  fi
fi

echo ""
echo "2) Can pyrealsense2 open it?"
/opt/miniconda3/bin/python3 - <<'PY'
import pyrealsense2 as rs
ctx = rs.context(); devs = ctx.query_devices()
print(f"   devices listed: {len(devs)}")
for d in devs:
    try:
        print("   ✓ opens OK →", d.get_info(rs.camera_info.name),
              "| USB", d.get_info(rs.camera_info.usb_type_descriptor),
              "| fw", d.get_info(rs.camera_info.firmware_version))
        print("   READY:  python3 fish_detection.py")
    except Exception as e:
        print("   ✗ listed but won't open:", e)
        print("     (matches a USB-2 / hung-interface link — fix the cable above.)")
PY
echo "============================================================"
