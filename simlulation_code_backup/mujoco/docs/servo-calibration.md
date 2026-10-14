# Servo calibration notes

Transcribed from the handwritten sheet in docs/images/tick_measurement.jpeg and corrected by hand.
All values are servo ticks. Direction (which sign is "up", "left", "open") is not yet
recorded; see the open questions at the bottom.

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

## Open questions

- S1: was 85° meant, or 90°? The scaled 275 ticks assumes the 85° reading is right.
- S2: confirm start value 467 vs 407.
- For every joint: which tick direction is which physical direction (left/right, up/down, open/close)?
- S4: minus 90° from home would be below 0 ticks, so the home pose sits at one end of the wrist range. Confirm the usable tick range per servo (0 to 1000? 0 to 4095?).
