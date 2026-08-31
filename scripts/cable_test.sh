#!/bin/zsh
# LIVE USB-3 cable tester (uses the RealSense's negotiated link speed).
# Run this, then plug the D435i in THROUGH the cable you want to test.
# It reports whether the link came up at USB 3 (good cable) or USB 2 (bad /
# charge-only cable, or a marginal contact -- try flipping the connector).
# No camera permission needed (this reads USB link info, not the camera).
NAME="Intel(R) RealSense(TM) Depth Camera 435i"
echo "==================================================================="
echo " Live USB link tester — plug the camera in through your test cable."
echo " Flip the connector or swap cables to compare.   Ctrl-C to stop."
echo "==================================================================="
prev="__init__"
while true; do
  TREE=$(ioreg -p IOUSB -w 0 -l -r -n "$NAME" 2>/dev/null)
  if [ -z "$TREE" ]; then
    line="•  no D435i seen  (unplugged, or cable not seated at all)"
  else
    LINK=$(echo "$TREE" | grep -m1 UsbLinkSpeed | grep -oE '[0-9]+$')
    NIF=$(echo "$TREE" | grep -c IOUSBHostInterface)
    case "$LINK" in
      480000000)   line="✗  USB 2.0  (480 Mb/s)  — NOT a USB-3 link   [interfaces=$NIF]";;
      5000000000)  line="✓  USB 3.0  (5 Gb/s)    — USB-3 cable, GOOD  [interfaces=$NIF]";;
      10000000000) line="✓  USB 3.1  (10 Gb/s)   — USB-3 cable, GOOD  [interfaces=$NIF]";;
      "")          line="?  linked but speed unreadable                [interfaces=$NIF]";;
      *)           line="?  link=$LINK bps                              [interfaces=$NIF]";;
    esac
  fi
  if [ "$line" != "$prev" ]; then
    echo "  [$(date +%H:%M:%S)]  $line"
    prev="$line"
  fi
  sleep 1.5
done
