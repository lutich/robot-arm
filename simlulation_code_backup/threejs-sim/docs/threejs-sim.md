# three.js simulator: what it does, how, and what comes next

The recommendation in `HANDOVER.md` section 5 was to drop Unreal for a small web tool.
That tool now exists in `threejs-sim/`. It uses three.js for drawing and our own code for everything else; the Unreal project and the MuJoCo scripts were removed.
How to run it is in `threejs-sim/README.md`.
This document explains the parts, the decisions behind them, and the next steps, including how it gets into the controller's web page.

![Simulator](images/threejs-sim.png)

## 1. Principles

- **The schematic is canon.** All joint distances come from `docs/images/arm-dimensions.png` via `threejs-sim/js/config.js`. The rendered model was measured in the browser and matches (section 3).
- **The sim behaves like the controller.** Same units, guards, rejection rules and trajectory shape as `lutich/robot-arm`, so a plan that runs in the sim is a plan the controller accepts.
- **Nothing reaches the real arm by accident.** Read only by default, motion only through the controller's own API, after a sim check, with explicit operator consent (section 8).
- **No heavy toolchain.** Plain ES modules and a vendored three.js. Node is only used for the dev server and the tests; Python only for the existing optional `vision/` scripts.

## 2. Architecture

```
            browser (threejs-sim/index.html)
 ┌──────────────────────────────────────────────────────────────┐
 │ app.js  UI, main loop                                        │
 │  ├ controller.js  emulated controller (counts, smoothstep)   │
 │  ├ world.js       arm pose, grasp rules, physics stepping    │
 │  │   ├ kinematics.js  FK / IK / counts <-> angles            │
 │  │   ├ collision.js   arm shapes, self/board contacts        │
 │  │   └ physics.js     cube rigid bodies                      │
 │  ├ planner.js     tasks -> poses -> verified plan            │
 │  ├ camview.js     Camera tab: calib.js, detect.js            │
 │  ├ live.js        HTTP client of the controller API          │
 │  └ model.js / render.js   three.js scene                     │
 └───────────────▲──────────────────────────────────────────────┘
                 │ same origin
   server.mjs: static files + mock controller  |  read-only proxy to the Pi
               or the controller itself, with the patch in integration/
```

## 3. Model and dimensions

| Check on the rendered model | Schematic | Measured in the browser |
|-----------------------------|-----------|-------------------------|
| Board top to S2 | 9,6 | 9,6 |
| S2 to S3 | 10,5 | 10,5 |
| S3 to S4 | 12,6 | 12,6 |
| S4 to flange | 6,7 | 6,7 |
| Flange height, arm straight up | 39,4 | 39,4 |
| Pad gap open / closed | 4,5 / 0 | 4,5 / 0 |
| Flange to pads, open / closed | 10,0 / 11,5 | 10,0 / 11,5 |

Kit drawings found on the web confirm the brackets (`docs/images/web/SOURCES.md`):

- **Upper arm:** two long U brackets bolted base to base give 2 × 50 + 2 × 2 = 104 mm horn to horn. That matches the measured 10,5.
- **Base:** two "U beams" of 90 × 42 × 25 mm form the 9 × 8,4 plate.
- **Horns:** Ø20, with 4 × M3 holes on a Ø14 circle.

From the photos (`docs/images/photos/`):

- The S1 servo stands at the back of the base plate, which reaches forward over the mat.
- S3 rides on the forearm, and the upper arm is screwed to its horn.
- The PCA9685 sits on pink foam on the upper arm.
- The servo cables are routed per link and follow the pose.

**Mat offset.** The overhead photo `setup-top-view-base.jpg` shows the board's back edge at about mat X = 10 mm. With S1 1,5 cm in front of that edge, d = −2,5 cm. The mat paper overhangs the board there and folds down. This is a photo estimate, not a measurement (open item 3).

## 4. Controller emulation

`controller.js` copies `control/domain/motion.py` and `docs/api.md`:

- **Motion:** one motion at a time. All joints share one cubic smoothstep, whose duration is 1,5 × the largest Δ / speed. Moves, jogs and HOME/PARK run at 20 counts/s; demo playback at 1 to 100.
- **Limits:** targets outside the guards are rejected, never clamped.
- **Stop:** switches the outputs off.
- **Same object, three uses:** the page runs it, `server.mjs --mock` serves it over the same HTTP API, and the planner verifies plans with it headless.

The HTTP API moves one joint at a time (`move`), so the sim has two playback modes:

- **Demo playback:** all joints together, like the controller's own "Run all once".
- **Single-joint moves:** the only way to reach a computed pose over the API. For each step, the planner tries joint orders and keeps the first one whose intermediate poses stay clear of everything.

## 5. Collision and physics

**Arm collision.** Every arm part is a capsule or a box sized from the photos. Self collision is tested between links that are not neighbours; arm parts are also tested against the board. Both can stop the motion like an emergency stop.

**Cubes are rigid bodies** (`physics.js`, about 400 lines, no library):

- **Contacts:** a separating axis test, then face clipping for box-box contact manifolds (up to 4 points), vertex tests for the board and desk planes, and closest points for capsule-box.
- **Solver:** sequential impulses with accumulated clamping, Coulomb friction in two tangent directions, speculative contacts, Baumgarte correction, and 240 Hz substeps.
- **Arm as a collider:** the arm is kinematic. Its surface velocity comes from its motion between substeps, so it can push and topple cubes.
- **Sleeping:** bodies that stay still go to sleep, so stacks stand quietly.
- **Tested:** a 3-stack stays put, a tumbling drop lands flat, a 75 % overhang topples, and the arm sweeping through a cube pushes it.

Grasping is rule based on purpose: two-finger friction grasps are not stable in a small solver. When the closing jaws reach a cube between the pads, they square it up (up to 20° off), centre it, and hold it rigidly. On release it becomes a dynamic body with the hand's velocity. The log reports the S6 count where the jaws touch the cube (≈ 567); higher counts squeeze it.

## 6. Planning

`planner.js` turns "blue on orange", "grün auf blau" or "yellow to 0 14" into servo counts:

1. Top-down IK (`docs/pick-and-place-plan.md` section 3), with S5 chosen so the jaws are square to the cube and inside the S5 guards.
2. The sequence of plan section 4, with vertical descent and lift in 1 cm steps (0,25 cm for single-joint moves). The hover is 5 cm on the mat and 2 cm above stacks, and drops if out of reach.
3. Where a joint-space transit would hit a stack, one or two raised via poses are inserted.
4. The whole plan runs headless in a copy of the world, with physics. It passes only if there is no contact and the cube ends within 0,9 cm of the target, flat.

## 7. Camera: simulation, calibration, recognition

- **Simulated view:** the OAK camera is rendered with its exact pinhole model (focal length and principal point as a custom projection matrix) and its fitted pose, with the real frame overlaid.
- **Calibration** (`calib.js`), from the mat markers:
  1. Normalised DLT homography.
  2. Focal length from the homography's orthogonality constraints.
  3. Pose from its columns.
  4. Levenberg-Marquardt over pose, focal length and principal point.

  On the saved frame it reproduces OpenCV's fit from `vision/blocks.py`: f = 950 px, principal point (655, 360), camera at (481, −8, 372) mm, 0,88 px RMS. For a new frame, clicking the 6 marker centres is enough. The result moves the camera in the 3D model.
- **Cube recognition** (`detect.js`):
  1. Classify pixels by HSV colour, keeping only those whose viewing ray hits the mat.
  2. Find connected components.
  3. Back-project each blob to half cube height. Its mean is the cube's position, and its principal axis the yaw.
  4. Check the size against a projected 26,5 mm cube.

  "Put them into the sim" places the cubes on the mat, and tasks can then go to them. It assumes cubes lie on the mat. In the saved frame the orange cube is actually held in the gripper, so detection reports it at the mat point below.

**Live video and tracking.** The Camera tab can show the controller's MJPEG stream (`/api/camera/stream.mjpg`, read only).
"Track continuously" runs every registered tracker on the current frame.
The cube tracker can also move the sim's cubes to what the camera sees.
On 2026-10-12 against the real camera it ran at the camera's 5 fps, about 17 ms per frame (`images/live/2026-10-12-1707-live-video-tracking.png`).
The tracker hook (`addTracker`) is where the gripper ArUco tracker for the HOME-angle calibration goes.

**HOME check.** The controller's own HOME illustration (`images/controller-ui/home-illustration.png`) shows S2 at about 22°, S3 at about 70° and S4 at about 88° in the model's convention. The fit to the taught demo gave 6°, 62° and 84°.
The illustration is not to scale (its link ratio is 0,94 against the real 0,83), so neither value is a measurement.
A live check already agrees on S1: with S1 = 505 counts the sim arm and the real arm in the camera frame point the same way.

## 8. Safety model and the shared API

The server is the arm: `/api` is the controller's low-level API, answered by a backend switched for everybody (`docs/arm-api.md`). The page is one client of it.

| Situation | What can happen |
|-----------|-----------------|
| `server.mjs` | Everything, on the shared simulated arm. A collision stops it like an emergency stop; `power_on` re-arms it |
| `server.mjs --controller <pi>` | The real arm can be selected, read only: state and `/api/stop` pass, every other POST gets HTTP 409 |
| `server.mjs --controller <pi> --allow-motion` | Real mode moves the arm. Every motion call is first played on a copy of the simulation synced to the real arm's state and refused (409) if the arm would hit itself or the board. `stop` is never blocked. The page shows a red banner and frame, and asks once before a plan or scenario |
| Plans and scenarios | The page's planner computes each trajectory in the simulation (single-joint moves, as the API sends them) and sends it to the active backend; the server checks every call again |
| Emergency stop button | `POST /api/stop` to the active backend |

## 9. Integration into the controller's web page

**Goal:** open the simulator from the controller's own page at `http://<pi>:8765/`, view the real arm next to the sim, and later control it from there.

**Option A, no change on the Pi.** Run `server.mjs --controller http://<pi>:8765` on a laptop. The proxy sends the matching Origin, which is the only way a page served elsewhere can talk to the API. Read only unless `--allow-motion`.

**Option B, served by the controller.** `integration/robot-arm-sim-route.patch` (+34 lines in `lutich/robot-arm`):

- adds a read-only `/sim/` route that serves the `threejs-sim` folder when `ROBOT_ARM_SIM_DIR` is set;
- adds a "3D simulator" button in the page header, shown only when the route exists.

Same origin means no proxy, and the controller's own checks apply unchanged. The patch compiles and applies to commit `3792d64`. It has not been run on the Pi and is not applied to the controller repo, because that repo belongs to the team and needs their review (`integration/README.md`).

**Later, Option C.** Embed the 3D view in the controller page itself (an `<iframe src="/sim/?embed=1">` next to the joint panel). The sim would follow `/api/state`, and a "Preview" button would show a demo's path before "Run all once".

## 10. Next steps, in order

1. **Measure HOME angles and directions** (open item 1). Until then the sim uses angles fitted to the taught demo. With the gripper ArUco marker (`tracking-plan.md` section 4), the camera calibration here can measure them: command poses, detect the marker, fit q_home and dir per servo.
2. **Measure d**, or fit it from an S1 sweep seen by the camera.
3. **Agree on Option B with the team.** Then the simulator ships with the controller.
4. **Teach plans as demos.** The controller can only record demos from the current pose (`demo/add`). A plan can be taught step by step (single-joint moves, then `demo/add` at each pose) and then replayed with all joints together at demo speed. That is much smoother than 200 single moves.
5. **Detect stacked cubes in the browser** too, with the level logic of `vision/blocks.py`, and refresh the sim scene from the camera before each task.
6. **Closed loop:** after each place, compare the camera's cube positions with the sim and correct the next plan.
