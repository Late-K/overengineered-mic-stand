# Bill of materials

Prices are rough USD from AliExpress/Amazon-type sellers (late 2026). Search for the **spec** column, not a brand. Belt lengths are confirmed from CAD, so order them last or buy a small assortment.

## Electronics (~$140)

| Qty | Part | Spec to search for | ~$ |
|---|---|---|---|
| 1 (+1 spare) | Raspberry Pi Pico | "Raspberry Pi Pico H" (pre-soldered headers) | 5 |
| 4 (+1 spare) | Stepper driver | "TMC2209 v1.3 BTT" or "FYSETC TMC2209", UART version | 25 |
| 3 | Stepper motor (arm) | "17HS4401" NEMA17, 40 mm body, 1.7 A, 0.40 N·m, 5 mm shaft | 30 |
| 1 | Wrist motor | "NEMA11 planetary gearbox stepper 27:1": 28 mm, ~45 mm body (~0.12 N·m before the gearbox), ~0.67 A. Keep the motor + gearbox at about 200 g or less | 25 |
| 3 | Motor dampers | "NEMA17 rubber vibration damper" | 6 |
| 4 | Magnetic encoder | "AS5600 encoder module" (comes with a magnet; buy spares) | 8 |
| 5 | Encoder magnets | "diametric magnet 6x2.5mm" (spares, glued into printed shaft caps) | 3 |
| 1 | I²C multiplexer | "TCA9548A module" | 2 |
| 1 | Distance sensor | "VL53L1X breakout" (Pololu, Adafruit, or a GY-53 / CJMCU-531 clone) | 10 |
| 1 | Power supply | 24 V 3 A DC brick, 5.5×2.1 mm barrel plug | 13 |
| 1 | Panel jack + fuse | 5.5×2.1 mm panel-mount jack, inline 3 A blade fuse holder | 3 |
| 1 | Buck converter | "MP1584 buck module" (set to 5.0 V before connecting) | 2 |
| 1 | Schottky diode | 1N5819 (buck → Pico VSYS) | 0.5 |
| 4 | Capacitors | 100 µF 35 V electrolytic, one at each driver's VM | 1 |
| 2 | Buttons | 12 mm momentary push buttons (panel mount) | 2 |
| 1 | Status LED | "WS2812B 5mm" or a single WS2812 module, + 330 Ω resistor | 1 |
| — | Resistors | 1 kΩ (UART TX), 2× 2.2 kΩ (I²C pull-ups) | 0.5 |
| — | Wiring | JST-XH connector kit, 22 AWG (power) + 26 AWG (signal) silicone wire, 7×9 cm perfboard, heat-shrink | 10 |
| 1 | Shielded cable | ~1.5 m thin 4-core shielded cable (I²C up the arm: ToF sensor + wrist encoder) | 3 |
| 1 | Motor cable | ~1.5 m thin 4-core flexible (silicone) cable for the wrist motor, twisted pairs | 3 |
| 1 | *Optional* USB isolator | "ADuM3160 USB isolator": only if the stand causes hum in your audio | 10 |

## Motion hardware (~$55)

| Qty | Part | Spec | ~$ |
|---|---|---|---|
| 3 | Motor pulleys | GT2 20T, 5 mm bore, 6 mm belt | 4 |
| 3 | Intermediate pulleys | GT2 16T, 8 mm bore (2nd stage; or print them) | 4 |
| 3 + 3 | Large pulleys | 60T and 96T GT2: **print these** in PETG (20T→60T, then 16T→96T = 18:1) | — |
| 6 | Belts | GT2 6 mm **closed loop**; lengths from CAD (typically 200–300 mm) | 8 |
| 6 | Idlers | 624ZZ bearings (belt tensioners and cord pulleys) | 3 |
| 12 | Pivot bearings | 608ZZ (two per pivot, spaced apart) | 6 |
| 1 | Yaw bearing | 6806-2RS thin-section (30×42×7) **or** 100 mm lazy-susan bearing | 6 |
| 2 | Yaw thrust | 608ZZ or F8-16M axial bearing | 2 |
| — | Shafts | 8 mm steel rod 300 mm (cut to length) or M8 shoulder bolts | 5 |
| 4 | Rod ends | M5 rod-end bearings (push-rod and levelling links) | 4 |
| 2–4 | Arm springs | Extension springs, **~1.1 N/mm** (shoulder) and **~1.0 N/mm** (elbow), max load ≥ 160 N, ≥ 150 mm extension. Two half-rate springs side by side also work. Confirm with the sim for your payload | 10 |
| 1–2 | Wrist spring | Small extension spring (~0.3–0.5 N/mm, ≥ 60 mm extension) to carry ~85% of the mic's weight at the wrist; final size set during tuning | 3 |
| 2 | Wrist bearings | 688ZZ (8×16×5) or 6700 thin-section, either side of the wrist shaft | 3 |
| 3 m | Cord | 1.5 mm Dyneema / UHMWPE cord (doesn't stretch or creep) | 3 |
| 3 | Turnbuckles | M4 (fine spring adjustment: shoulder, elbow, wrist) | 3 |

## Structure (~$50)

| Qty | Part | Spec | ~$ |
|---|---|---|---|
| 2 | Main links | Carbon fibre tube **20 mm OD × 18 mm ID, 500 mm** (cut to 400 mm centre-to-centre). Aluminium 20×1.5 mm tube is the budget option | 18 |
| 2 | Push-rod / levelling links | Carbon fibre rod or tube 8–10 mm, 500 mm | 6 |
| 1 | Desk clamp | Steel C-clamp from a monitor arm ("monitor arm desk clamp replacement"), rated ≥ 10 kg / ≥ 30 N·m | 12 |
| — | Fasteners | M3/M4/M5 socket-head assortment, M8 bolts + nyloc nuts, washers | 10 |
| — | Inserts | M3 and M4 brass heat-set inserts | 5 |
| 1 kg | Filament | PETG (or ASA). PLA creeps under constant spring load | 15 |
| — | Glue | 2-part epoxy (tubes into printed joints) | 5 |

## Mic accessories (~$25)

| Qty | Part | Spec | ~$ |
|---|---|---|---|
| 1 | Shock mount | Lightweight spider shock mount for ~50 mm diameter mics (fits the C-1), 5/8″ thread | 10 |
| 1 | Thread adapter | 3/8″ male → 5/8″ female | 1 |
| 1 | Pop filter | Clip-on foam windscreen or a small shock-mount pop filter (lighter than a gooseneck) | 6 |
| 1 | XLR cable | 3 m, thin and flexible | 6 |
| 1 m | Sleeving | 10 mm braided cable sleeve | 2 |

## Tools to buy, if you don't have them (~$45)

| Tool | Why |
|---|---|
| Digital calipers | Measuring printed fits and bearings |
| Multimeter | Setting the buck converter to 5.0 V, checking wiring |
| JST-XH crimper (e.g. "SN-28B") | Motor and sensor connectors |
| Heat-set insert tip for the soldering iron | Inserts in the printed parts |
| Fine-tooth hacksaw or rotary tool + **dust mask** | Cutting carbon tube. Cut it wet; carbon dust is harmful |

**Total:** about **$220–$265** excluding tools (the flip-up wrist adds about $40).
