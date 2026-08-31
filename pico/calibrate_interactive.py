#!/usr/bin/env python3
"""
Foolproof wheel-direction calibration for the fish-car.

Spins each wheel FORWARD, one at a time, and asks whether it rolled the car
forward. No left/right/front/back labels to track. Prints the exact INVERT line
to drop into pico/main.py.

  *** WHEELS OFF THE GROUND ***
  Pick ONE end of the car as "the FRONT" (the camera / fish-tank end) and keep
  it consistent the whole time — that's the only reference you need.

Run:  python3 pico/calibrate_interactive.py
"""
import glob
import sys
import time

import serial

WHEELS = ["FL", "FR", "RL", "RR"]


def find_port():
    for pat in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def spin(ser, wheel, secs):
    end = time.time() + secs
    while time.time() < end:
        ser.write(f"M {wheel} 0.30\n".encode())
        time.sleep(0.05)
    ser.write(b"S\n")


def main():
    port = find_port()
    if not port:
        sys.exit("No Pico serial port found — plug the Pico into this computer.")
    ser = serial.Serial(port, 115200, timeout=0)
    time.sleep(0.3)

    print("=" * 57)
    print(" WHEEL DIRECTION CALIBRATION")
    print("=" * 57)
    print(" * Wheels OFF the ground.")
    print(" * Pick which end of the car is the FRONT (camera end) and")
    print("   keep it fixed the whole time.")
    print(" * For each wheel: does the TOP of the wheel spin toward the")
    print("   FRONT?  (that is what rolls the car forward)")
    print("=" * 57)

    invert = {}
    for i, wheel in enumerate(WHEELS, 1):
        input(f"\n[{i}/4] Press Enter to spin a wheel FORWARD...")
        while True:
            print("   spinning 4s — watch the TOP of the wheel...")
            spin(ser, wheel, 4.0)
            ans = input("   Top moved toward the FRONT? [y]es / [n]o / [r]epeat: ").strip().lower()
            if ans.startswith("r"):
                continue
            invert[wheel] = ans.startswith("n")
            break

    ser.write(b"S\n")
    ser.close()
    line = "INVERT = {" + ", ".join(f'"{w}": {invert[w]}' for w in WHEELS) + "}"
    print("\n" + "=" * 57)
    print(" DONE — replace the INVERT line in pico/main.py with:")
    print("   " + line)
    print("=" * 57)


if __name__ == "__main__":
    main()
