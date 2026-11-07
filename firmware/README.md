# Mic arm firmware (Raspberry Pi Pico)

C++ for the RP2040, built with PlatformIO and the Earle Philhower Arduino core. Core 0 runs the state machine, sensors and USB serial. Core 1 does nothing except generate step pulses, so motion stays smooth whatever else is going on.

| File | What it does |
|---|---|
| `src/config.h` | **Every** pin, dimension, limit, speed and current. Most changes happen here only |
| `src/main.cpp` | State machine, deploy/stow routes, buttons, LED, serial commands |
| `src/axes.*` | TMC2209 setup (StealthChop, currents) and minimum-jerk moves on core 1 |
| `src/encoders.*` | AS5600 joint encoders through the TCA9548A mux |
| `src/kinematics.*` | Arm geometry (same maths as `sim/arm_sim.py`) |
| `src/tracking.*` | VL53L1X distance sensor: filtering, deadband, nudges |
| `src/safety.*` | Following-error, joint-limit and StallGuard checks |
| `src/storage.*` | Calibration and taught pose saved as JSON in flash |
| `tools/soak_test.py` | Deploys and stows 50 times and checks repeatability |

## Build and flash

```bash
pip install platformio
```
```bash
pio run
```
To flash, hold **BOOTSEL** on the Pico while plugging it in, then:
```bash
pio run -t upload
```
On Windows, if the first build fails with "Filename too long", run this once and try again:
```bash
git config --global core.longpaths true
```

## Wiring

| Pico pin | Goes to |
|---|---|
| GP6 / GP7 | Yaw driver STEP / DIR |
| GP8 / GP9 | Shoulder driver STEP / DIR |
| GP10 / GP11 | Elbow driver STEP / DIR |
| GP2 / GP3 | Wrist driver STEP / DIR |
| GP12 | EN on all four drivers (LOW = on) |
| GP13 / GP14 / GP15 / GP26 | DIAG of yaw / shoulder / elbow / wrist driver |
| GP4 (UART1 TX) | through a **1 kΩ** resistor to PDN_UART on all four drivers |
| GP5 (UART1 RX) | PDN_UART on all four drivers (direct) |
| GP16 / GP17 | I²C SDA / SCL to the TCA9548A (2.2 kΩ pull-ups to 3.3 V) |
| GP18 | Deploy/Stow button to GND |
| GP19 | Teach button to GND |
| GP22 | WS2812 data in (330 Ω in series) |
| VSYS | 5 V from the buck converter through a 1N5819 Schottky diode |

- **Driver UART addresses** are set with MS1/MS2 on each board: yaw = both LOW (0), shoulder = MS1 HIGH (1), elbow = MS2 HIGH (2), wrist = both HIGH (3).
- **Mux channels:** 0 yaw encoder, 1 shoulder encoder, 2 elbow encoder, 3 VL53L1X, 4 wrist encoder. The ToF sensor and the wrist encoder share one cable up the arm.
- **The wrist driver** sits in the electronics box with the others. Only the motor and encoder cables run up the arm.
- **Every driver** needs a 100 µF capacitor across VM/GND, right at the board.
- **Never plug or unplug a motor while 24 V is on.** It destroys TMC2209s.

## First power-up

1. **Before fitting any belts,** check each motor turns with `JOG`.
   - Motor current is set over UART by `RUN_CURRENT_MA`; the Vref pot on the driver boards is ignored.
   - If a joint turns the wrong way, flip its `MOTOR_DIR` in `config.h`.
2. Fit the belts. Move the arm **by hand** into its folded park, resting on its stops: upper arm up, forearm folded down beside it, mic flipped up onto the wrist stop. Then send `CAL`. This records where each magnet is in that pose, so the firmware never needs to home.
3. Move a joint by hand and check `STATUS?` shows the right sign.
   - If it doesn't, flip that joint's `ENC_DIR` and run `CAL` again.
4. Press **Teach**: the motors go limp. Drag the mic to where you want it and press **Teach** again to save the spot.
5. The aim for your desk (`SET aim=82`) is already the default. If you change `sim/config.json`, run the sim and send the `SET aim=` value it prints. Then `SET dist=150` (or your preferred distance).
6. Send `DEPLOY` and `STOW` a few times. Then run the soak test:
   ```bash
   python tools/soak_test.py
   ```

## Serial protocol

USB serial, 115200 baud, one command per line. Commands are case-insensitive.

| Command | Does |
|---|---|
| `DEPLOY` | Folded → unfold (turn, raise and flip the mic down in one move) → 100 mm above target → target, then tracks your distance |
| `STOW` | The same route backwards, from wherever the arm is |
| `TEACH` | 1st: motors limp. 2nd: save the current pose as the target |
| `STOP` | Stop now (→ `FAULT`, motors braked) |
| `STATUS?` | Replies with a `STATE` line |
| `CAL` | Arm is resting folded on its stops: store the encoder zero |
| `SET dist=<mm>` | Mic-to-face distance to hold (80–400) |
| `SET aim=<deg>` | Which way the mic faces relative to the arm (from the sim) |
| `JOG j=<0..3> deg=<deg>` | Move one joint (0 yaw, 1 shoulder, 2 elbow, 3 wrist). Only in `FAULT` or `LIVE` |

**Replies** are `OK`, `ERR <reason>`, or:
```
STATE <STOWED|DEPLOYING|LIVE|STOWING|TEACH|FAULT> yaw=<deg> sh=<deg> el=<deg> wr=<deg> dist=<mm> [reason=<text>]
```
A `STATE` line is also sent by itself on every state change. `dist` is `-1` when nobody is in front of the sensor. Yaw runs from 180 to 420: past 360 means the arm has swung round in front of the main desk. Wrist 0 = mic hanging, about 185 = folded up.

**Buttons:**
- **Deploy/Stow** toggles between deployed and stowed.
- **Teach** works like the `TEACH` command.
- **Either button pressed while the arm moves stops it.**

**LED:**

| Colour | State |
|---|---|
| Dim blue | Stowed |
| Amber | Moving |
| Green | Live |
| Purple | Teach |
| Blinking red | Fault |

## Safety behaviour

- **Following error.** Every 20 ms the encoders are compared with where the motors were told to be. A difference over `MAX_FOLLOW_ERR_DEG` (3°) means a collision, a missed step or a bump. The arm then goes to `FAULT`: current is cut and the coils are shorted, so it resists gently and sinks slowly instead of pushing.
- **Speed limits.** Moves are planned so that no point of the arm or mic goes faster than 600 mm/s away from you, 150 mm/s near you, 75 mm/s on the final drop, or 50 mm/s for tracking nudges.
- **Distance tracking** never moves the mic more than 190 mm either side of the taught spot (your 38 cm lean). If you get closer than 80 mm it backs off immediately.
- **Silent wrist.** The wrist motor brakes at zero current whenever it isn't moving, including while LIVE. A hanging mic is stable by itself, and when folded it leans on its stop.
- **StallGuard** is off by default (`SG_THRESHOLD = 0`). To use it:
  1. Set `SG_THRESHOLD` to around 50. Higher values trip more easily.
  2. Deploy and stow repeatedly. If it trips with nothing in the way, lower the value and try again.
  3. Once it stops false-tripping, lower it a further 10 to leave a margin.
- **Clearing a FAULT:** send `DEPLOY` or `STOW`. Both start by re-reading the encoders.
