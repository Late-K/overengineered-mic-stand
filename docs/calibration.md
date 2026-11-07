# Calibration and tuning

Do these in order. Use a serial terminal for the commands: the PlatformIO monitor (`pio device monitor`), PuTTY, or the Arduino IDE serial monitor, at 115200 baud.

## 1. Encoder magnets

- Each AS5600 must sit centred under its magnet (within ~0.5 mm), 1–2 mm away.
- At power-up, the firmware reports `encoder magnet missing` if one can't see its magnet.
- Turn each joint slowly by hand and watch `STATUS?`. The angle should change smoothly, with no jumps.

## 2. Zero the encoders (`CAL`)

1. Move the arm by hand into its **folded park**, resting on its stops:
   - the yaw against its parked-end stop (arm pointing across the side desk towards the far wall)
   - the upper arm up, the forearm folded down beside it
   - the mic flipped up and leaning on the wrist's folded stop
2. Send `CAL`. The firmware now knows where every magnet is in that pose, and saves that to flash.
3. Move each joint by hand and check `STATUS?`:
   - Lifting the shoulder **increases** `sh`.
   - Swinging the arm round towards your chair **increases** `yaw`.
   - Flipping the mic from hanging towards folded **increases** `wr`.
   - If a joint reads backwards, flip its `ENC_DIR` in `config.h`, re-flash, and repeat `CAL`.
4. `JOG j=1 deg=5` should lift the shoulder, and `JOG j=3 deg=-10` should start lowering the mic from folded. If a joint goes the other way, flip its `MOTOR_DIR`.

You only redo `CAL` if you move a magnet or an encoder board.

## 3. Counterbalance springs

The springs carry the weight; the motors should only have to fight friction.

The sim prints the spring rate needed for your payload. For this design that is ≈1.12 N/mm (shoulder) and ≈0.97 N/mm (elbow), with the cord pulley 50 mm above the pivot and the cord attached 80 mm out. That geometry behaves like a "zero-length" spring, which balances at **every** angle when the tension is right.

1. Send `TEACH` so the motors go limp. Everything else gets the full payload: mic, shock mount, pop filter, cable.
2. **Shoulder first.**
   - Hold the upper arm horizontal and let go.
   - Adjust the turnbuckle until the arm **drifts down slowly**: a few seconds to fall 10 cm.
   - It must **not** rise. Slightly under-balanced is deliberate: if power fails, the arm sinks away from your face.
3. Check the shoulder at 45° up and at 30° down. It should behave the same.
   - If it's balanced at one angle but not another, the pulley isn't straight above the pivot, or the 50/80 mm distances are off.
4. **Then the elbow**, the same way, with the upper arm held still.
5. **Then the wrist**, with the arm held still and the forearm level:
   - Lift the mic to sideways (90°) and let go. It should swing **slowly back to hanging**, not drop.
   - Lift it to near straight up and let go. It should lean over onto the folded stop.
   - If it drops fast, tighten the wrist spring. If it stays put or creeps up at 90°, loosen it. Slightly under-balanced is deliberate: the mic must always hang by itself when live, because the wrist motor is switched off then.
6. Press Teach again (it saves the current pose, which you'll redo in step 5) or send `STOW`.

## 4. Motor current

- `RUN_CURRENT_MA` in `config.h` defaults to 1200 mA. Lower it until deploys still never fault, then add ~20%. Less current means cooler and quieter motors.
- Motors should be warm, never too hot to touch.
- The drivers need airflow, or a heatsink if they sit in a closed box.

## 5. Teach your spot

1. Sit how you normally sit.
2. Press **Teach** (LED purple, motors limp) and drag the mic to where you want it.
3. Press **Teach** again. The LED turns green and the spot is saved.
4. If you get `ERR no room to approach from above`, the arm can't fit 100 mm above that spot. Choose a spot a little lower or closer to the clamp.

## 6. Distance tracking

1. The firmware's default aim (82°) is the value the sim printed for your desk. If you change `sim/config.json`, run `python sim/arm_sim.py` and send the `SET aim=<n>` value it prints. This tells the firmware which way the mic faces relative to the arm.
2. Send `SET dist=150`, or whatever capsule-to-mouth distance sounds best to you. 10–15 cm is typical for the C-1 with a pop filter.
3. With the arm LIVE, `STATUS?` shows `dist=`. It should read your distance to within ~1 cm. It reads `-1` when nobody's there.
4. Lean back 10 cm and wait. After ~2 s the arm should follow slowly. Lean in and it backs off.
   - If it moves sideways instead, the `aim` value is wrong. Adjust it in steps of 10.

## 7. Audio check

The part that matters most.

1. Set up the C-1 as for a real recording. Turn the interface gain **up** (~+20 dB more than normal).
2. Record 30 s of the arm **stowed and idle**.
3. Record a deploy, then 30 s of the arm **LIVE and still**.
4. Lean back to trigger a tracking nudge and record that too.
5. Listen on headphones and look at a spectrogram (Audacity: View → Spectrogram).

| What you hear | Likely cause | Fix |
|---|---|---|
| Steady whine while still | Holding current or chopper noise | Lower `HOLD_FRAC`; make sure StealthChop is active (`en_spreadCycle(false)`) |
| Hum at 50/60 Hz only when the stand is plugged in | Ground loop through USB | Fit an ADuM3160 USB isolator |
| Ticks or creaks while moving | Belt teeth, a dry bearing, or a cable rubbing | Re-tension, grease, re-route cables |
| Thumps through the mic | Vibration reaching the capsule | Check the shock-mount bands aren't touching; lower `TIP_ACCEL` |
| Whine or buzz from the head while LIVE | The wrist motor is holding with current | It should brake at zero current when still; check the wrist driver's UART address is 3 and it answers at boot |
| Rubbing or tugging sound when the mic moves | The XLR loop at the wrist is too tight | Give the cable more slack at the wrist |

Tracking nudges are slow and short, but if you record in a very quiet room you can set `TRACK_DEADBAND_MM` larger so it moves less often.

## 8. Soak test

```bash
python firmware/tools/soak_test.py --cycles 50
```
It passes when every deploy and stow lands within 1° of the first one, with no FAULT.
