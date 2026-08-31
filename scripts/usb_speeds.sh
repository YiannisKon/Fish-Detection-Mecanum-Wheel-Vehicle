#!/bin/zsh
# List every USB device and the speed its link negotiated.
# Use it to test a cable with ANY USB-3 device (a flash drive / SSD): plug that
# device in through the cable in question and see if IT links at USB 3. If a
# known USB-3 stick also shows USB 2 through this cable, the CABLE is USB-2.
# If the stick shows USB 3 but the camera shows USB 2, the issue is the camera
# (or its port), not the cable.
echo "USB devices and negotiated link speed:"
echo "--------------------------------------"
ioreg -p IOUSB -w 0 -l 2>/dev/null | awk '
  /\+-o / { name=$0; sub(/.*\+-o /,"",name); sub(/@.*/,"",name) }
  /"UsbLinkSpeed"/ {
    spd=$NF; n++
    if (spd==480000000)       lbl="USB 2.0  (480 Mb/s)"
    else if (spd==12000000)   lbl="USB 1.1  (12 Mb/s)"
    else if (spd==5000000000) lbl="USB 3.0  (5 Gb/s)  <== USB-3"
    else if (spd==10000000000)lbl="USB 3.1  (10 Gb/s) <== USB-3"
    else                      lbl=spd" bps"
    printf "  %-46s %s\n", substr(name,1,46), lbl
  }
  END { if (n==0) print "  (no USB devices attached right now — plug one into a USB-C port)" }'
echo "--------------------------------------"
echo "480 Mb/s = the cable/link is USB 2.   5 Gb/s or 10 Gb/s = USB 3."
