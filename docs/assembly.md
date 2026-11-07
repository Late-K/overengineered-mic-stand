# Assembly

The order matters. Get the mechanics moving smoothly **by hand** and balanced **before** any electronics go on. A stiff or unbalanced arm is the main cause of noisy, stalling motion. Add photos here as you build.

## Printing

- **PETG** (or ASA), 0.2 mm layers, **4 walls**, 5 top/bottom layers, 40% gyroid infill.
- Print pulleys and joint hubs so the bore axis is vertical, which keeps the holes round.
- Holes for bearings: design them 0.1–0.15 mm under size and press-fit. Do a test print of one 608 seat first.
- Holes for heat-set inserts: follow the insert maker's size (usually 4.0 mm for M3, 5.6 mm for M4).
- Fits that should slide: 0.2–0.3 mm clearance.

## 1. Clamp and yaw turret

1. Bolt the printed **offset bracket** to the steel C-clamp. It puts the yaw axis ~60 mm out from the side desk's front edge, which gives the arm room to swing past the edge.
2. Press the 6806 (or lazy-susan) bearing into the turret base, with the thrust bearing under it.
3. Fit the yaw motor on its rubber damper. Belt stage 1: 20T → 60T. Stage 2: 16T → 96T, on the turret ring.
4. Glue a diametric magnet into the centre of the turret shaft. Fit the yaw AS5600 board on a fixed bracket 1–2 mm below it, centred within 0.5 mm.
5. Print the yaw hard stops at the two ends of its 240° range, with rubber bumpers:
   - **Parked end:** the arm points straight across the side desk towards the far wall.
   - **Other end:** 240° round from there, past the direction of your chair.

   The exact numbers are `LIMIT_MIN` / `LIMIT_MAX` in `firmware/src/config.h`.
6. **Check:** the turret spins freely with no rocking when you push on it sideways.

## 2. Shoulder housing (sits on the turret)

1. Mount the shoulder motor and elbow motor side by side on dampers.
2. Assemble both 18:1 belt reductions:
   - The **shoulder** drive turns the upper-arm hub.
   - The **elbow** drive turns a crank on the same axis. A push-rod from the crank runs to the elbow, so the forearm angle is set from the base.
3. Each output hub gets two 608 bearings, spaced as far apart as the housing allows.
4. Put a magnet in each output shaft, with an AS5600 facing it, both fixed to the housing.
5. Fit the levelling link: the second parallelogram that keeps the mic head level, anchored to the turret.
6. Tension the belts until they "twang" with no slack. **Don't over-tighten:** that loads the bearings and makes noise.

## 3. Arm links and wrist head

1. Cut the carbon tubes to length, wet, wearing a mask. Centre-to-centre spacing is 400 mm for both links (or whatever your sim run chose).
2. Epoxy the tubes into the printed joint ends. **Check alignment on a flat surface while the epoxy cures.**
3. Join the elbow with 608 bearings and an 8 mm shaft. Connect the push-rod (M5 rod ends) and the levelling link.
4. The **head bracket** stays level via the levelling link. It carries the wrist about 90 mm past the forearm's end. That gap leaves room for the flipped-up mic to sit beside the forearm.
5. The **wrist**:
   - **Motor:** the NEMA11 + 27:1 gearbox mounts on the head bracket. Its output shaft runs horizontally, across the arm's plane, in two 688ZZ bearings.
   - **Mic mount:** a short printed wrist arm on the shaft carries the 3/8″→5/8″ adapter, with a lockable tilt knob, then the shock mount and C-1.
   - **ToF sensor:** the VL53L1X sits on the wrist arm, pointing where the mic points.
   - **Encoder:** a magnet goes in the end of the wrist shaft, with the wrist AS5600 facing it on the bracket.
   - **Hard stops:** one just past hanging (about −5°), and the **folded stop** just past straight up (about 185°), both padded. When folded, the mic leans on that stop, so the wrist never holds it up with current.
6. **Check:**
   - Move the arm through its whole range by hand. Nothing binds, the head stays level, and the push-rod never goes straight (dead centre).
   - Flip the mic by hand. It moves smoothly from hanging to folded, clears the forearm, and settles on the folded stop.

## 4. Springs

See [calibration.md](calibration.md#3-counterbalance-springs).
- Each spring pulls a Dyneema cord running over a 624-bearing pulley mounted **straight above** the joint's pivot (50 mm above by default).
- The cord attaches 80 mm out along the upper arm, or along the elbow crank for the forearm.
- A turnbuckle in each cord sets the tension.
- **Wrist spring:** a small spring on its own cord, anchored to the head bracket. It carries most of the mic's weight as it flips, the same way the arm springs work.
- Cover the springs. A broken spring or cord must not be able to whip out.

## 5. Electronics

The full pin table is in [firmware/README.md](../firmware/README.md#wiring). Exact pin numbers live in `firmware/src/config.h`.

1. **Set the buck converter to 5.0 V with the multimeter BEFORE connecting it to the Pico.**
2. Power path: 24 V jack → 3 A fuse → driver VM pins (a 100 µF capacitor at each driver) and → buck → 1N5819 → Pico VSYS.
3. TMC2209 UART:
   - All four PDN_UART pins are tied together.
   - From there, one wire goes straight to GP5 (RX), and another goes through a 1 kΩ resistor to GP4 (TX).
   - Set each driver's address with MS1/MS2: yaw 0, shoulder 1, elbow 2, wrist 3 (both HIGH).
4. STEP/DIR: yaw GP6/7, shoulder GP8/9, elbow GP10/11, wrist GP2/3. EN pins: all to GP12. DIAG pins: GP13/14/15, wrist GP26.
5. I²C: Pico GP16/GP17 → TCA9548A, with 2.2 kΩ pull-ups to 3.3 V. Mux channels:
   - 0: yaw encoder
   - 1: shoulder encoder
   - 2: elbow encoder
   - 3: the ToF sensor, through the shielded cable up the arm
   - 4: the wrist encoder, on the same cable
6. Buttons to GP18 / GP19 and GND. WS2812 data to GP22 through 330 Ω.
7. Mount everything in a printed box on the turret, with the USB port and 24 V jack on the outside.

**Cable runs:**
- Twist each motor's two coil pairs and keep them away from the XLR cable.
- Run the XLR, the shielded I²C cable and the wrist motor cable together in braided sleeving along the arm.
- Clip the cables to each link, with a loose service loop at every joint, so they can't snag at any pose.
- **The wrist flips the mic through ~185°.** Give the XLR a generous loop there, and check it doesn't pull or rub in either end position.

## 6. First power-up

Follow [firmware/README.md → First power-up](../firmware/README.md#first-power-up), then [calibration.md](calibration.md).
