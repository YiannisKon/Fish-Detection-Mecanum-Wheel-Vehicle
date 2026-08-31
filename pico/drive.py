#!/usr/bin/env python3
"""
Hold-to-drive keyboard console for the fish-car (SSH-friendly).

HOLD keys to move; RELEASE and the car ramps to a stop (gentle, tank-safe).
COMBINE keys for diagonals: W+D = forward-right, S+A = back-left, etc.
(Terminals don't report key-release, so this tracks per-key auto-repeat:
keys seen within the last HOLD_S seconds count as 'held'.)

KEYS (hold):
  W / S   forward / back            A / D   strafe left / right
  Q / E   forward-left / forward-right corner
  Z / C   back-left / back-right corner
  J / L   rotate CCW / CW
  + / -   speed scale up / down     1/2/3/4 jog FL/FR/RL/RR forward (calibration)
  SPACE   stop (gentle ramp)        x       quit (sends stop first)

If holding stutters (pulsing), raise your OS key-repeat rate or HOLD_S.
"""
import glob
import select
import sys
import termios
import time
import tty

import serial

BAUD = 115200
SEND_HZ = 25
HOLD_S = 0.45           # a key counts as held this long after its last repeat
ORDER = ("FL", "FR", "RL", "RR")
MOVE_KEYS = "wsadqezcjl"


def find_port():
    for pat in ("/dev/ttyACM*", "/dev/cu.usbmodem*", "/dev/tty.usbmodem*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def main():
    port = find_port()
    if not port:
        print("No Pico serial port found (ttyACM / usbmodem).")
        return 1
    ser = serial.Serial(port, BAUD, timeout=0)
    print(f"connected: {port}")
    print(__doc__)

    scale = 0.5
    last_seen = {}          # key -> time of last repeat event
    jog_wheel = None
    jog_seen = 0.0
    period = 1.0 / SEND_HZ

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            now = time.time()
            while select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1).lower()
                if k in MOVE_KEYS:
                    last_seen[k] = now
                    jog_wheel = None
                elif k in "1234":
                    jog_wheel = ORDER[int(k) - 1]
                    jog_seen = now
                    last_seen.clear()
                elif k == " ":
                    last_seen.clear()
                    jog_wheel = None
                elif k in "+=":
                    scale = min(1.0, scale + 0.1)
                elif k == "-":
                    scale = max(0.1, scale - 0.1)
                elif k == "x":
                    raise KeyboardInterrupt

            def held(key):
                return now - last_seen.get(key, 0) < HOLD_S

            if jog_wheel and now - jog_seen >= HOLD_S:
                jog_wheel = None

            if jog_wheel:
                ser.write(f"M {jog_wheel} {scale:.2f}\n".encode())
                status = f"JOG {jog_wheel} @ {scale:.2f}"
            else:
                vx = scale * (held("w") + held("q") + held("e")
                              - held("s") - held("z") - held("c"))
                vy = scale * (held("a") + held("q") + held("z")
                              - held("d") - held("e") - held("c"))
                wz = scale * (held("j") - held("l"))
                mag = (vx * vx + vy * vy) ** 0.5
                if mag > scale:          # corners capped at the same speed
                    vx, vy = vx / mag * scale, vy / mag * scale
                ser.write(f"V {vx:.2f} {vy:.2f} {wz:.2f}\n".encode())
                moving = "MOVING " if (vx or vy or wz) else "stopped"
                status = f"{moving} vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} spd={scale:.1f}"

            back = ser.read(200).decode(errors="replace").strip()
            sys.stdout.write(f"\r{status:58s}{('<'+back) if back else '':<10}")
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            ser.write(b"S\n")
            ser.close()
        except Exception:
            pass
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
