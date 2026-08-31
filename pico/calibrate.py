#!/usr/bin/env python3
"""
Motor DIRECTION calibration for the fish-car.

Spins each wheel FORWARD then REVERSE, ONE AT A TIME, with a live countdown so
you can watch each wheel by itself. Note any wheel whose "FORWARD" spins the
wrong way (i.e. would drive the car backward) and flip that wheel's INVERT
flag in pico/main.py, then re-upload the firmware.

  *** PUT THE WHEELS OFF THE GROUND before running ***

Run:  python3 pico/calibrate.py
      python3 pico/calibrate.py 0.35 3     # optional: duty, seconds
"""
import glob
import sys
import time

import serial

WHEELS = ["FL", "FR", "RL", "RR"]
DUTY = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5


def find_port():
    for pat in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def hold(ser, cmd, secs, label):
    end = time.time() + secs
    while time.time() < end:
        ser.write((cmd + "\n").encode())
        print(f"\r   {label:22s} {end - time.time():3.0f}s ", end="", flush=True)
        time.sleep(0.05)
    print()


def main():
    port = find_port()
    if not port:
        sys.exit("No Pico serial port found — plug the Pico into this computer.")
    ser = serial.Serial(port, 115200, timeout=0)
    time.sleep(0.3)
    print(f"connected: {port}")
    print("*********** WHEELS OFF THE GROUND ***********\n")
    time.sleep(1.0)

    for w in WHEELS:
        print(f"========== {w} ==========")
        hold(ser, f"M {w} {DUTY:.2f}", SECS, f"{w}  >> FORWARD")
        hold(ser, "S", 1.0, f"{w}  stop")
        hold(ser, f"M {w} {-DUTY:.2f}", SECS, f"{w}  << REVERSE")
        hold(ser, "S", 1.5, f"{w}  stop")
        print()

    ser.write(b"S\n")
    ser.close()
    print("Done.  Which wheels spun the WRONG way on FORWARD?")
    print("Flip the INVERT flags for those wheels in pico/main.py and re-upload.")


if __name__ == "__main__":
    main()
