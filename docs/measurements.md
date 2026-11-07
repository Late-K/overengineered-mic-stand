# Measurement sheet

Fill this in before designing anything, then copy the numbers into `sim/config.json`. To create that file:
```bash
python sim/arm_sim.py --write-config
```
The **config key** column says where each number goes.

## The sim's coordinate frame

The **origin** is the shoulder pivot. It sits on the yaw axis, at the top of the clamp turret.

| Axis | Direction |
|---|---|
| **x** | Across the desk. Positive = away from the desk, out past its side edge |
| **y** | Along the desk's side edge. Positive = towards you (the front of the desk) |
| **z** | Up |

All lengths are in millimetres. Measure with the chair where you normally sit.

## Weights (kitchen scale)

| What | Value | Config key |
|---|---|---|
| Mic (Behringer C-1) | ____ g | |
| Shock mount | ____ g | |
| Pop filter / windscreen | ____ g | |
| XLR cable, the part that hangs from the arm (~0.5 m) | ____ g | |
| ToF sensor + head bracket (estimate 40 g) | ____ g | |
| **Payload total** | ____ kg | `mass_kg.payload` |

## Desk

| What | Value | Config key |
|---|---|---|
| Floor to desk top | ____ | `desk.top_height` |
| Desk top thickness | ____ | `desk.thickness` |
| Desk depth (front to back) | ____ | `desk.depth` |
| Where the clamp goes: distance from the desk's **front** edge, along the side | ____ | `desk.front_y` |
| Anything under the desk at the clamp spot (drawers, frame, crossbar)? How far down is it clear? | ____ | check the stowed pose fits |
| Clearance needed over the desk (keyboard, mug, monitor foot) | ____ (default 80) | `desk.clearance_over_desk` |

## You

Sit normally and measure the position of your mouth from the clamp spot:

| What | Value | Config key |
|---|---|---|
| x: how far **in** from the desk's side edge (enter as a **negative** number, and subtract a further 60 mm for the clamp offset) | ____ | `user.mouth[0]` |
| y: how far **in front of the clamp spot**, along the side edge | ____ | `user.mouth[1]` |
| z: mouth height **above the desk top**, minus 80 (heights are measured from the shoulder pivot, which sits 80 mm above the desk) | ____ | `user.mouth[2]` |
| Distance you want between your mouth and the capsule | ____ (default 150) | `user.distance` |
| Where the mic should sit relative to your mouth: straight ahead and a little low is `[0, -1, -0.25]` | ____ | `user.approach` |

## Mic hanging geometry (after you have the shock mount)

| What | Value | Config key |
|---|---|---|
| Arm tip (mount thread) to capsule centre | ____ (default 120) | `geometry.mic_drop` |
| Arm tip to the lowest point of the mic / shock mount | ____ (default 220) | `geometry.mic_hang` |

After entering the numbers, run:
```bash
python sim/arm_sim.py
```
Fix any FAIL before starting CAD. `--sweep` finds link lengths that work for your desk.
