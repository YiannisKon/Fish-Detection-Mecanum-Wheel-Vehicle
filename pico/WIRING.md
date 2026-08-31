# Fish-car wiring — Pico W ↔ 2× H-bridge drivers ↔ 4 mecanum wheels

Motor labels (viewed from above, "front" = camera/forward direction):
`FL` front-left · `FR` front-right · `RL` rear-left · `RR` rear-right

Driver control header silkscreen (per channel): **V P1 A1 B1 G  /  V P2 A2 B2 G**
- V = logic power (3.3 V)   G = logic ground
- P = PWM (speed)   A = INA (dir)   B = INB (dir)   [1 = channel 1, 2 = channel 2]

---

## 1. Pico W GPIO → Driver control pins  (12 signal wires)

| Wheel | Pico GP | Pico phys pin | Driver | Driver header pin |
|------|--------|---------------|--------|-------------------|
| FL PWM | GP0  | 1  | A | P1 |
| FL INA | GP1  | 2  | A | A1 |
| FL INB | GP2  | 4  | A | B1 |
| FR PWM | GP3  | 5  | A | P2 |
| FR INA | GP4  | 6  | A | A2 |
| FR INB | GP5  | 7  | A | B2 |
| RL PWM | GP6  | 9  | B | P1 |
| RL INA | GP7  | 10 | B | A1 |
| RL INB | GP8  | 11 | B | B1 |
| RR PWM | GP9  | 12 | B | P2 |
| RR INA | GP10 | 14 | B | A2 |
| RR INB | GP11 | 15 | B | B2 |

## 2. Logic power (Pico → BOTH drivers)  — 3.3 V, opto-isolated side

| From Pico | To |
|-----------|----|
| **3V3(OUT)** phys pin 36 | Driver A **V** (both V pins) **and** Driver B **V** |
| **GND** phys pin 38 (or 3/8/13/…) | Driver A **G** (both G pins) **and** Driver B **G** |

⚠️ V must be **3.3 V** (matches Pico's 3.3 V signals). Do **not** feed 5 V here.
If the two drivers pull more than the Pico's 3V3 rail likes, power V from the
**Pi's 3.3 V pin** instead (Pi GND = Pico GND via the USB cable, so it's fine).

## 3. Motor outputs (driver → wheels)

| Driver / channel | Terminal | Wheel motor |
|------------------|----------|-------------|
| A ch1 | M1_A / M1_B | FL |
| A ch2 | M2_A / M2_B | FR |
| B ch1 | M1_A / M1_B | RL |
| B ch2 | M2_A / M2_B | RR |

(If a wheel spins backwards during calibration: swap its two motor wires **or**
flip its `INVERT` flag in the firmware — no rewiring needed.)

## 4. Motor battery (high-current side — ISOLATED from logic)

| Battery | To |
|---------|----|
| **+** (through a fuse + main switch) | Driver A **P+** and Driver B **P+** |
| **−** | Driver A **P−** and Driver B **P−** |

- Battery voltage must match your motors, within **6.5–28 V**.
- **Do NOT connect battery − to Pico/Pi GND** — the opto-isolation keeps the
  motor side separate on purpose. Logic ground and motor ground stay separate.
- Datasheet: add an inline **fuse**; don't short motor outputs.

## 5. USB / ports

| Cable | From | To |
|-------|------|----|
| USB webcam | webcam | Pi 5 USB-A (blue = USB3, preferred) |
| Pico link + power | Pico W USB | Pi 5 USB-A (any) → shows up as `/dev/ttyACM0` |
| Pi power | Pi 5 USB-C | official Pi 5 power supply (5 V/5 A) |

## Ground summary (important)
- **Logic ground domain:** Pi GND = Pico GND (via USB) = driver **G** pins. Common.
- **Motor ground domain:** battery **−** = driver **P−** only. Separate/isolated.
- Keep the two domains apart; the driver bridges them safely via opto-isolators.

## First-power checklist (before motors move)
1. Wheels **off the ground** (up on a box) for the very first test.
2. Motor battery **off** while wiring; logic (USB) on first to flash/verify.
3. Power motors only after the Pico firmware is confirmed idling (motors stopped).
