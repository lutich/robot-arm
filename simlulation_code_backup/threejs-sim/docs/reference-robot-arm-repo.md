# Reference: lutich/robot-arm (controller software)

Repository: https://github.com/lutich/robot-arm, read on 2026-10-12 at commit `3792d64`.
Local clone for the team machine: `C:\Users\adm_bjung\Documents\robot-arm`, next to this repo, not inside it.

This is the control software for our arm: a Raspberry Pi drives the servos through a PCA9685 board.
The browser app in our screenshots (`images/screenshot_from_ui_controls.png`) is this repo's `web/manual_control.html`.
The repo does no coordinate (XYZ) control and has no physics simulation; its own README says so.
That is the gap this repo, `px-robotic-arm-sim`, fills.

Everything below is copied from that repo's files.
The repo itself marks most values as observations, not validated limits.

## 1. Facts we take over

### Units: "ticks" are PCA9685 counts

| Item | Value | Source |
|------|-------|--------|
| Unit of every servo value | PCA9685 counts at a nominal 60 Hz PWM frequency | `config/reference-arm/*.json`, `docs/api.md` |
| Time per count | 1 / 60 Hz / 4096 = 4,07 µs | Calculated |
| Channels | 0 to 5 = S1 to S6 | `docs/hardware.md` |
| Position feedback | None, the servos only receive commands | `docs/hardware.md` |

Plausibility check: a typical MG996R covers 180° over about 500 to 2500 µs, so about 2000 µs / 4,07 µs = 491 counts per 180°, or 2,73 counts per degree.
Our measured 2,44 to 3,06 ticks per degree (`servo-calibration.md`) fit this.

### Count guards in the software

The app rejects targets outside these ranges (`src/roboter_arm/shared/joints.py`).
They are historical guards from the old config, not measured safe travel.

| Servo | Channel | Joint | Low | High |
|-------|---------|-------|-----|------|
| S1 | 0 | Base rotation | 90 | 660 |
| S2 | 1 | Shoulder | 185 | 660 |
| S3 | 2 | Elbow | 90 | 660 |
| S4 | 3 | Wrist flexion | 90 | 660 |
| S5 | 4 | Wrist rotation | 90 | 660 |
| S6 | 5 | Gripper | 490 | 660 |

### HOME and PARK

| Pose | S1 | S2 | S3 | S4 | S5 | S6 | Source |
|------|----|----|----|----|----|----|--------|
| HOME | 320 | 427 | 403 | 220 | 480 | 650 | `config/reference-arm/home.json`, saved 2026-10-01, same as our `servo-calibration.md` |
| PARK | 330 | 540 | 580 | 195 | 480 | 650 | `config/reference-arm/park.json`, the supported rest pose before power on and after power off |

Power on moves PARK, then HOME. Power off moves back to PARK and switches the outputs off.

### Historical sign per servo

The old config (`config/reference-arm/inherited.json`) has a `sign` per servo.
Its meaning is not documented; it may be the direction of increasing counts relative to some convention.
It is a hint for our direction question, not an answer.

| S1 | S2 | S3 | S4 | S5 | S6 |
|----|----|----|----|----|----|
| -1 | -1 | +1 | -1 | -1 | -1 |

## 2. HTTP API (how a computed pose reaches the arm)

The app runs on the Pi at `http://raspberrypi.local:8765/`, Swagger docs at `/docs`.
On the team network it was at http://10.102.4.103:8765/ on 2026-10-12; what it showed then is in `controller-live-2026-10-12.md`.
Full description: `docs/api.md` in that repo. The relevant actions, all `POST /api/...` with a JSON body:

| Action | Body | Use for us |
|--------|------|------------|
| `move` | `{"channel": 2, "target": 410}` | One joint at a time, others hold |
| `go_pose` | `{"name": "HOME"}` | Back to the calibrated start |
| `demo/add`, `demo/run` | | Record the current pose, play all poses |
| `demo/load` | `{"filename": "<32 hex>.json"}` | Load a saved demo file |
| `stop` | `{}` | Emergency stop, outputs off |

Safety rules of the API that matter for us:

- POSTs must carry `Origin: http://<host>` and `Content-Type: application/json`, otherwise 403.
- Targets outside the current bounds are rejected, not clamped.
- Manual moves run at 20 counts/s; demo playback at the demo speed (1 to 100 counts/s).
- There is no position feedback, so the API reports commanded counts only.

The app also has a preview mode without hardware (`uv run --locked python scripts/manual_control.py`, then `http://127.0.0.1:8765/`).
The README lists macOS and Linux for it. It simulates commands only, no physics.

## 3. Camera: already calibrated to the table

The repo has an OAK camera (RGB plus stereo depth) and a saved pixel-to-table mapping, `config/camera-table-xy/calibration.json`, fitted on 2026-10-11.
This is the base for the "camera for fine adjustments" step in `pick-and-place-plan.md`.

| Item | Value |
|------|-------|
| Reference | Printed mat, 280 x 360 mm, 6 ArUco markers (`DICT_4X4_50`), 10 mm grid. PDF in `output/pdf/table-reference-a4.pdf` |
| Mapping | Homography from 1280 x 720 RGB pixels to table X, Y in mm, valid only on the mat surface (Z = 0) |
| Accuracy | 0,64 mm RMS, 1,06 mm max on six check points |
| Validity | Only while camera, focus (lens position 65) and mat stay where they were |

### Camera frame vs. our board frame

The camera frame is not the frame of `arm-dimensions.md`:

| | Camera table frame | Our board frame |
|-|--------------------|-----------------|
| Origin | Midpoint of the mat edge nearest the arm | Board top under the S1 axis |
| Toward the front, away from the arm | +X | +Y |
| Sideways | +Y = left, seen from the arm | +X = right, seen from the arm |
| Unit | mm | cm |

Conversion, with `d` = distance from the S1 axis to the mat's near edge (not measured yet):

```
x_board = -Y_cam / 10
y_board = (X_cam / 10) + d
```

Both frames are right-handed with Z up.

## 4. What the repo does not have

- No measured joint angles, angle zeros or link lengths. Ours are in `arm-dimensions.md` and `servo-calibration.md`.
- No inverse kinematics or XYZ control. Ours is in `threejs-sim/js/kinematics.js`.
- No physics simulation. Ours is in `threejs-sim/` (`js/physics.js`, `js/world.js`).
