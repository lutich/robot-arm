# Changelog

Branch `feature/threejs-sim`. Newest first. Dates are when the change was committed.

## 2026-10-13 (night): Taller base, tag layout checked on a fourth pose, measuring guide

- **Base:** the top plate is now 3,3 cm high (from the photos). The S1 shaft and the S2 axis keep their heights. Model, collision and photo fit use `config.js BASE`.
- **Planner:** when no single-joint order is free of contacts, it goes up through the raised arm, across and down.
- **Tag layout:** a fourth live pose confirms it (the S5 axis 0,5° / 0,14 cm from the model's, without the one weak pose).
- **[`docs/measure-arm-tags.md`](docs/measure-arm-tags.md):** what to measure on the arm (S5 hub to tag 107, and so on), with an annotated photo.

## 2026-10-13 (evening): Joints combine tags across parts; S5 axis calibrated; full-screen options

- **Joint fit** (`js/jointfit.js`):
  - Per frame, the wrist pose and S5 (7 numbers) come from the tags of both parts.
  - Across 3 or more recorded poses, the real S5 axis is fitted. That corrects the tag layout and measures S5's offset and direction (Record pose, Fit S5 axis).
- **Tag layout calibrated:** from two live snapshot pairs and the close-up photos, the S5 axis is 10,6° / 0,5 cm from the model's. The wrist + gripper pair fit improved from 17–50 px to 3–9 px.
- **Camera views** (in the tab, the Cameras view and full screen) have a toolbar: simulation, mat, markers, tags, parts, ghost, frame opacity, track, full screen.
- Tag axes are made orthonormal everywhere. A tag's quarter turn is reported only on clear evidence.
- **📷 Cubes from cameras** (task card): one snapshot per live camera, cubes located (in 3D with both cameras) and put into the simulation at their stack levels.
- **Demo finale:** only a few cubes are dropped (green on top, red clipping the edge, one thrown), at 1× speed; the speed returns to the previous value afterwards.
- **Gripper model:** straight finger bars, as in the photos; the pad positions are unchanged (measured).
- **Open:** the distance from the S5 hub to tag 107 (the joint motion cannot show it). See [`docs/two-camera-tracking.md`](docs/two-camera-tracking.md).

## 2026-10-13 (late afternoon): Rigid-part tracking with tags 106–109, both cameras side by side

- **Tags on the arm:**
  - Wrist: 109 (S5 servo face) and 108 (S5 bracket side plate).
  - Gripper: 107 and 106 (two faces of the S6 servo).
  - Their layout on each part is measured from close-up photos. All earlier tag photos were removed. Tags need a white margin.
- **Rigid parts** (`js/rigidtrack.js`): each part's pose comes from all its tags in all cameras at once.
  - Each camera is weighted by its calibration quality (the OAK counts more than the webcam).
  - Single-tag poses are refined and the mirror ambiguity is resolved.
  - Each tag's in-plane turn is found automatically, and "Learn tag layout" measures where a tag sits.
- **Camera tab:** both cameras side by side, the estimated parts drawn as wireframes over the frames, small tags searched 2× enlarged.
- **Main view:** "Cameras" shows both cameras instead of the 3D view; ⛶ for full screen, one or both.
- **Model:** the S5 servo is back in its original orientation (the earlier 90° turn was wrong).
- **On live snapshots:** wrist 2,5 px, gripper 1,5 px. See [`docs/two-camera-tracking.md`](docs/two-camera-tracking.md) for checks and the next step: anchor and HOME calibration from a few poses.

## 2026-10-13 (afternoon): Two cameras, arm tracking with AprilTags, cubes in 3D

- **Second camera.** The webcam on the monitor is the controller's `/api/camera2/*` (status, snapshot.jpg, stream.mjpg, capture), next to the OAK's `/api/camera/*`. `server.mjs --camera <controller>` forwards both, GET only.
- **Camera tab with both cameras.**
  - A camera selector; each camera has its own frame source and its own calibration from the mat markers ("Calibrate all cameras").
  - The 3D view shows both cameras and their fields of view, and a "Webcam" view button.
- **Arm tracking.** AprilTags 36h11 are on the arm: 72 on the S5 servo, 71 on the S6 servo.
  - They are detected in both cameras' frames and triangulated (rays from both views intersected).
  - The joint angles are fitted to the tag views. The tracked pose shows as a ghost arm, with "tracked − commanded" per joint.
  - Code: `js/armtrack.js`, codes in `js/apriltag36h11.js`.
- **Cubes in 3D.** Cubes seen by two cameras are located where their viewing rays meet, corrected from the visible faces to the cube's centre.
  - Stacked cubes and a cube in the gripper get their real height; they are no longer assumed to lie on the mat.
  - "Put them into the sim" places them at their stack level. Code: `js/locate3d.js`.
- **Model:** the tags on the arm with their real patterns, and the webcam on its monitor.
- **Cube detector:** pale, brightly lit faces now join a cube's blob (hysteresis), so top faces count.
- Details and checks: [`docs/two-camera-tracking.md`](docs/two-camera-tracking.md). 35 tests.
- Not yet: real frames from both cameras; the tag positions are measured from photos only; the simulated backend serves no camera2 frames.

## 2026-10-13: One robot API for the simulated and the real arm

**For other teams: you can now develop against the simulated arm with exactly the API of the real controller, and switch to the real arm without changing your code.**

### Start it

```powershell
git checkout feature/threejs-sim
cd threejs-sim
node server.mjs                     # Node 24, no npm install needed; http://127.0.0.1:8770/
node server.mjs --host 0.0.0.0      # reachable from other machines on the network
```

Open http://127.0.0.1:8770/ to watch the simulated arm. Every program that talks to the server sees the same arm and the same cubes.

### Use it

The API is the controller's low-level API (`lutich/robot-arm` `docs/api.md`): `GET /api/state`, `POST /api/move`, `/api/jog`, `/api/go_pose`, `/api/stop`, `/api/power_on`, `/api/settings`, `/api/limits`, `/api/demo/*`.

```python
import json, urllib.request
BASE = "http://127.0.0.1:8770"

def call(path, body=None):
    req = urllib.request.Request(f"{BASE}/api/{path}", method="POST" if body is not None else "GET",
        data=None if body is None else json.dumps(body).encode(),
        headers={"Content-Type": "application/json", "Origin": BASE})
    return json.load(urllib.request.urlopen(req))

call("move", {"channel": 0, "target": 400})     # S1 to 400 counts
print(call("state")["commands"])
```

- POSTs need `Content-Type: application/json` and `Origin: <the server's address>`, the same rules as the real controller.
- Targets outside the limits are rejected (HTTP 400), and only one motion runs at a time.
- Full reference: [`docs/arm-api.md`](docs/arm-api.md).

### Simulation or real arm

- `GET /api/backend` tells which arm answers. `POST /api/backend {"target": "sim" | "real"}` switches it for everybody; the page has the same switch in its header.
- The server always starts on the simulation. The real arm is only available when the server runs with `--controller http://10.102.4.103:8765`, and it only moves with `--allow-motion` on top.
- In real mode, every call that can move the arm is first played in the simulation. If the arm would hit itself or the board, the call is refused with **HTTP 409** and the reason. `stop` is never blocked.
- In the simulation, a collision stops the arm like an emergency stop; `POST /api/power_on` re-arms it.
- The simulation's cubes: `GET /sim/world`, and `POST /sim/scene {"cubes": [["orange", 0, 14, 0, 0], ...]}` with [colour, x, y, level, yaw] in the board frame (cm). This is not part of the robot API.

### Also new

- The page is now a client of this API.
  - Joints, HOME/PARK, planned tasks and scenarios all send API calls to the active arm.
  - The 3D view mirrors the server's arm and cubes.
  - A red banner shows when the real arm is active.
- **Scenarios:** predefined block layouts with the steps to execute, or the cubes the camera sees plus typed tasks.
  - The planner turns each task into a trajectory; stored trajectories (the team's taught pick and place) are possible too.
  - ↺ Reset puts the cubes in place; ▶ Run executes the steps on the active arm.
- **Model vs. photo check:** the camera pose and joint angles are fitted to points marked in the reference photos (`tools/fit-views.mjs`, `tools/compare.mjs`).
  - The PARK pose fitted from five photos puts the gripper pads on the base plate, as in reality.
  - Results and open model differences: [`docs/model-photo-comparison.md`](docs/model-photo-comparison.md).
- **Model:** the PCA9685 board and its pink foam sit on the upper arm's side plate, as on the real arm.
- **Tests:** 29 (`node --test threejs-sim/test/*.test.mjs`), including the API backend with a second backend playing the real controller.

### Known limits

- The angle at HOME of each servo is still not measured, so the model's PARK puts the pads 2 mm into the board. At PARK the real-mode check therefore accepts gripper contact with the base and the board.
- Calibration edits in the page are local to that browser; the server's simulation uses `threejs-sim/js/config.js`.
- One camera through the API so far (`/api/camera/*`).

### Upgrading

Restart a running `server.mjs` after pulling. `--mock` is no longer needed (it is still accepted): the simulated arm is always there.

## 2026-10-12

- three.js simulator in `threejs-sim/`: kinematics, controller emulation, collision, cube physics, planning, camera calibration from the mat markers, cube recognition, tower demo (`docs/threejs-sim.md`).
- The MuJoCo and Unreal routes were removed.
- Reference photos of the arm, and a first render-vs-photo comparison tool.
