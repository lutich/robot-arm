# Servo calibration notes

Transcribed from the handwritten sheet in docs/images/tick_measurement.jpeg and corrected by hand.
All values are servo ticks. Direction (which sign is "up", "left", "open") is not yet
recorded; see the open questions at the bottom.

"Ticks" are **PCA9685 counts at a nominal 60 Hz**, about 4,07 µs per count.
That comes from the controller repo, see `reference-robot-arm-repo.md`.

## Joint names

| Servo | Joint          |
|-------|----------------|
| S1    | Base rotation  |
| S2    | Shoulder       |
| S3    | Elbow          |
| S4    | Wrist flexion  |
| S5    | Wrist rotation |
| S6    | Gripper        |

## Home position (we always start from here)

| S1  | S2  | S3  | S4  | S5  | S6  |
|-----|-----|-----|-----|-----|-----|
| 320 | 427 | 403 | 220 | 480 | 650 |

The controller repo's `config/reference-arm/home.json` has the same HOME.
**The live controller had a different HOME on 2026-10-12: S5 = 250, S6 = 550** (see `controller-live-2026-10-12.md`). Agree on one before relying on it.
It also has a PARK pose, the supported rest pose before power on and after power off:

| S1  | S2  | S3  | S4  | S5  | S6  |
|-----|-----|-----|-----|-----|-----|
| 330 | 540 | 580 | 195 | 480 | 650 |

## Count guards in the controller software

The app rejects any target outside these ranges. They are historical guards, not measured safe travel.

| Servo | Low | High |
|-------|-----|------|
| S1 | 90 | 660 |
| S2 | 185 | 660 |
| S3 | 90 | 660 |
| S4 | 90 | 660 |
| S5 | 90 | 660 |
| S6 | 490 | 660 |

## Measured 90° moves (raw, as written on the sheet)

| Servo | Joint          | From | To  | Delta | Angle | Ticks per degree | Notes |
|-------|----------------|------|-----|-------|-------|------------------|-------|
| S1    | Base rotation  | 320  | 60  | 260   | 85°   | 3.06             | Sheet says 85°, not 90° |
| S2    | Shoulder       | 467  | 247 | 220   | 90°   | 2.44             | First number corrected by hand, could be 407 |
| S3    | Elbow          | 543  | 303 | 240   | 90°   | 2.67             | First number corrected by hand |
| S4    | Wrist flexion  | 483  | 233 | 250   | 90°   | 2.78             | |
| S5    | Wrist rotation | 250  | 480 | 230   | 90°   | 2.56             | Ends exactly at home (480) |

## 90° moves starting from the home position

Delta for 90° is taken from the table above (S1 scaled from 85° to 90°: 260 / 85 * 90 = 275).
"Minus" and "plus" are tick directions, not yet mapped to a physical direction.

| Servo | Joint          | Home | Delta for 90° | Home minus 90° | Home plus 90° | Notes |
|-------|----------------|------|---------------|----------------|---------------|-------|
| S1    | Base rotation  | 320  | 275           | 45             | 595           | Measured move went 320 → 60 (minus direction) |
| S2    | Shoulder       | 427  | 220           | 207            | 647           | |
| S3    | Elbow          | 403  | 240           | 163            | 643           | |
| S4    | Wrist flexion  | 220  | 250           | -30 (not reachable) | 470      | Only the plus direction is possible from home |
| S5    | Wrist rotation | 480  | 230           | 250            | 710           | Measured move was 250 → 480, i.e. minus direction |

## Gripper (S6)

Home is 650 = max grip.

| Position   | Ticks | Opening (?) | Delta from home | Notes |
|------------|-------|-------------|-----------------|-------|
| Max grip   | 650   | 9           | 0               | Sheet shows 680 with 660 written next to it, corrected to 650 (home) |
| Block grip | 580   | 8,5         | -70             | |
| Min grip   | 490   |             | -160            | Leading character on the sheet crossed out, reads 490 |

The "Opening" column is the number written before the tick value (9 and 8,5).
No unit on the sheet, probably cm of gripper opening.

Direction, from the team's taught demo on the live controller: **490 = open, 590 = closed on the cube, 650 = fully closed**. Higher counts close the gripper.

## Open questions

- S1: was 85° meant, or 90°? The scaled 275 ticks assumes the 85° reading is right.
- S2: confirm start value 467 vs 407.
- For every joint: which tick direction is which physical direction (left/right, up/down, open/close)?
- S4: minus 90° from home would be below 0 ticks. With the software guard at 90, S4 can only move 130 counts (about 47°) in the minus direction from home.
- Several "home plus 90°" values exceed the 660 guard (S5: 710). S1 "home minus 90°" (45) is below the 90 guard. Those moves are rejected by the app as it is.
- The historical config has a sign per servo (S1 -1, S2 -1, S3 +1, S4 -1, S5 -1, S6 -1). Check whether it matches the physical directions.
- For every joint: the joint angle at the home tick. `pick-and-place-plan.md` needs it to turn computed angles into ticks.
