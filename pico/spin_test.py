#!/usr/bin/env python3
"""
Minimal ONE-motor spin test for the fish-car Pico firmware.

Spins the given wheel forward for a few seconds, then reverse, then stops.
Keeps sending at 20 Hz so the firmware's failsafe doesn't cut it off.
Run with the Pico plugged into THIS computer (Mac or Pi).

  python3 spin_test.py [wheel] [duty] [seconds]
  e.g.  python3 spin_test.py FL 0.30 3
wheel = FL | FR | RL | RR   (FL = driver channel 1)
"""
import glob
import sys
import time

import serial

wheel = sys.argv[1].upper() if len(sys.argv) > 1 else "FL"
duty = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0


def find_port():
    for pat in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def hold(ser, cmd, seconds):
    print(f"  -> {cmd}   ({seconds:.0f}s)")
    end = time.time() + seconds
    while time.time() < end:
        ser.write((cmd + "\n").encode())
        time.sleep(0.05)                      # 20 Hz keepalive


def main():
    port = find_port()
    if not port:
        sys.exit("No Pico serial port found (usbmodem / ttyACM).")
    ser = serial.Serial(port, 115200, timeout=0)
    print(f"port: {port}   wheel: {wheel}   duty: {duty}")
    time.sleep(0.3)
    ser.reset_input_buffer()
    try:
        hold(ser, f"M {wheel} {duty:.2f}", secs)      # forward
        hold(ser, "S", 1.0)                            # pause
        hold(ser, f"M {wheel} {-duty:.2f}", secs)     # reverse
    finally:
        ser.write(b"S\n")
        time.sleep(0.1)
        ser.close()
        print("done (stopped).")


if __name__ == "__main__":
    main()
