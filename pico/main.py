"""
Fish-car Pico W firmware  —  mecanum drive over USB serial.

Saved as main.py so it auto-runs when the Pico powers up (from the Pi's USB).
The Raspberry Pi sends short newline-terminated commands over /dev/ttyACM0;
this firmware mixes them into 4 wheel speeds and drives the two H-bridge boards.

PIN MAP (see pico/WIRING.md) — each motor = PWM + INA + INB:
    FL: GP0 GP1 GP2      FR: GP3 GP4 GP5
    RL: GP6 GP7 GP8      RR: GP9 GP10 GP11

SERIAL PROTOCOL (115200; USB-CDC ignores baud). One command per line:
    V <vx> <vy> <wz>   set body velocity, each -1..1
                       vx=forward+  vy=left+  wz=turn-CCW+
    S                  stop (ramp to 0)
    M <FL|FR|RL|RR> <d>  drive ONE motor at duty d (-1..1); others 0  [calibration]
    E <0|1>            disable / enable outputs (software kill switch)
    P                  ping -> replies "OK"
Replies: "FISHCAR ready" on boot, "OK" to P, "?" on a bad command.

SAFETY:
    * Failsafe watchdog: if no command arrives for CMD_TIMEOUT_MS, motors stop.
    * Slew-rate limiting ramps through zero (datasheet: don't snap-reverse at speed).
    * At boot, GPIOs float -> driver sees A=B=HIGH -> coast, so motors stay off
      until this firmware actively drives them.
"""
import sys
import time

try:
    import uselect
    from machine import Pin, PWM
    ON_PICO = True
except ImportError:            # lets mix() be imported/tested off-device
    uselect = None
    ON_PICO = False

# ----------------------------- config --------------------------------------- #
PWM_FREQ = 20000               # 20 kHz: above hearing, fine for these drivers
CMD_TIMEOUT_MS = 400           # failsafe stop if Pi goes quiet
LOOP_MS = 10
# Asymmetric ramp: speed up briskly, slow down GENTLY so the fish tank's water
# doesn't slosh on stops. (per 10 ms loop)
ACCEL_PER_LOOP = 0.035         # 0 -> full in ~0.29 s
DECEL_PER_LOOP = 0.010         # full -> 0 in ~1.0 s   (gentle stop curve)

MOTOR_PINS = {                 # (PWM, INA, INB)
    # Car FRONT/BACK definition flipped 180 deg (user turned the car around):
    # each label now maps to the wheel at that NEW position. The 180 flip is a
    # diagonal pin swap; the INVERT toggles cancel, so those stay unchanged.
    "FL": (9, 10, 11),
    "FR": (6, 7, 8),
    "RL": (0, 1, 2),
    "RR": (3, 4, 5),
}
ORDER = ("FL", "FR", "RL", "RR")
INVERT = {"FL": True, "FR": False, "RL": True, "RR": False}  # calibration 3 (180 flip)


# --------------------- mecanum mixing (pure / testable) --------------------- #
def mix(vx, vy, wz):
    """Body velocity -> [FL, FR, RL, RR] duties, normalized so max |w| <= 1."""
    fl = vx - vy - wz
    fr = vx + vy + wz
    rl = vx + vy - wz
    rr = vx - vy + wz
    m = max(1.0, abs(fl), abs(fr), abs(rl), abs(rr))
    return [fl / m, fr / m, rl / m, rr / m]


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def slew(cur, target):
    """One ramp step from cur toward target.

    Moving AWAY from zero (speeding up) uses ACCEL_PER_LOOP; moving TOWARD zero
    (stopping / reversing through zero) uses the gentler DECEL_PER_LOOP so the
    water in the tank isn't thrown around. Pure function -> unit-testable.
    """
    if target > cur:
        step = DECEL_PER_LOOP if cur < 0 else ACCEL_PER_LOOP
        return min(target, cur + step)
    if target < cur:
        step = DECEL_PER_LOOP if cur > 0 else ACCEL_PER_LOOP
        return max(target, cur - step)
    return cur


# ----------------------------- motor driver --------------------------------- #
class Motor:
    def __init__(self, pwm_pin, ina_pin, inb_pin, invert=False):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(PWM_FREQ)
        self.pwm.duty_u16(0)
        self.ina = Pin(ina_pin, Pin.OUT, value=0)
        self.inb = Pin(inb_pin, Pin.OUT, value=0)
        self.invert = invert
        self.cur = 0.0
        self.brake()

    def brake(self):
        self.ina.value(0)
        self.inb.value(0)
        self.pwm.duty_u16(0)
        self.cur = 0.0

    def apply(self, duty):
        d = -duty if self.invert else duty
        d = clamp(d, -1.0, 1.0)
        if d > 0.001:                       # forward: INA=L INB=H
            self.ina.value(0)
            self.inb.value(1)
            self.pwm.duty_u16(int(d * 65535))
        elif d < -0.001:                    # reverse: INA=H INB=L
            self.ina.value(1)
            self.inb.value(0)
            self.pwm.duty_u16(int(-d * 65535))
        else:                               # brake
            self.ina.value(0)
            self.inb.value(0)
            self.pwm.duty_u16(0)

    def step_toward(self, target):
        self.cur = slew(self.cur, target)
        self.apply(self.cur)


# ------------------- non-blocking USB-serial line reader -------------------- #
class LineReader:
    def __init__(self):
        self.poll = uselect.poll()
        self.poll.register(sys.stdin, uselect.POLLIN)
        self.buf = ""

    def readline(self):
        while self.poll.poll(0):
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                line = self.buf.strip()
                self.buf = ""
                if line:
                    return line
            elif ch is not None:
                self.buf += ch
                if len(self.buf) > 64:      # overflow guard
                    self.buf = ""
        return None


# --------------------------------- main ------------------------------------- #
def main():
    try:
        led = Pin("LED", Pin.OUT)
        led.value(1)
    except Exception:
        led = None

    motors = {n: Motor(*MOTOR_PINS[n], invert=INVERT[n]) for n in ORDER}
    reader = LineReader()
    targets = {n: 0.0 for n in ORDER}
    enabled = True
    last_cmd = time.ticks_ms()
    last_led = last_cmd

    print("FISHCAR ready")

    try:
        while True:
            now = time.ticks_ms()
            line = reader.readline()
            if line is not None:
                p = line.split()
                c = p[0].upper()
                try:
                    if c == "V" and len(p) == 4:
                        d = mix(float(p[1]), float(p[2]), float(p[3]))
                        for i, n in enumerate(ORDER):
                            targets[n] = d[i]
                        last_cmd = now
                    elif c == "S":
                        for n in ORDER:
                            targets[n] = 0.0
                        last_cmd = now
                    elif c == "M" and len(p) == 3 and p[1].upper() in targets:
                        for n in ORDER:
                            targets[n] = 0.0
                        targets[p[1].upper()] = clamp(float(p[2]), -1.0, 1.0)
                        last_cmd = now
                    elif c == "E" and len(p) == 2:
                        enabled = (p[1] == "1")
                        last_cmd = now
                    elif c == "P":
                        print("OK")
                        last_cmd = now
                    else:
                        print("?")
                except (ValueError, IndexError):
                    print("?")

            # failsafe: stop if disabled or the Pi went quiet
            if not enabled or time.ticks_diff(now, last_cmd) > CMD_TIMEOUT_MS:
                for n in ORDER:
                    targets[n] = 0.0

            for n in ORDER:
                motors[n].step_toward(targets[n])

            if led and time.ticks_diff(now, last_led) > 500:
                led.toggle()
                last_led = now

            time.sleep_ms(LOOP_MS)
    finally:
        for n in ORDER:                     # always stop on exit/exception
            motors[n].brake()
        if led:
            led.value(0)


if __name__ == "__main__" and ON_PICO:
    main()
