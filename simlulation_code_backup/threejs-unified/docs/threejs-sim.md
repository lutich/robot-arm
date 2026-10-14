# PX Three.js simulator architecture

`threejs-sim/` is the unified browser implementation of the PX robotic-arm digital twin. It keeps the richer kinematics, collision, physics, planning, camera, and detailed model from `feature/threejs-sim`, while presenting them through a polished dark responsive interface named **PX-robo-arm-simulator**.

The application is deliberately simulation-only. Controller terminology and count-based controls mirror the team’s existing robot UI for familiarity, but no module connects to or commands the physical arm.

## 1. Design principles

- **Preserve the measured geometry.** The dimensions in `docs/images/arm-dimensions.png` and the supplied reference photos determine the visible link proportions.
- **Preserve provenance.** Photo estimates, fitted calibration, historical bounds, observed poses, and measured dimensions must remain distinct.
- **Model controller semantics, not controller connectivity.** Counts, guards, HOME/PARK, jog, smoothstep motion, stop, and re-arm are simulated locally.
- **Check motion before playback.** Plans are replayed in a headless copy of the world and reported as passing or failing before the visible simulation runs them.
- **Stay offline.** Plain ES modules and vendored Three.js need only a local static server. There is no controller proxy or live-camera transport.

## 2. Runtime architecture

```text
browser (threejs-sim/index.html)
├── app.js             UI, scene editing, render/physics loop
├── controller.js      local count-based controller model
├── world.js           arm pose, grasping, contacts, physics state
│   ├── kinematics.js  FK, IK, count/angle conversion
│   ├── collision.js   arm primitives and collision tests
│   └── physics.js     cube rigid bodies
├── planner.js         task parsing, motion planning, verification
├── camview.js         simulated view and local reference images
│   ├── calib.js       marker-based camera fit
│   ├── aruco.js       browser marker detection
│   └── detect.js      coloured-cube recognition
├── model/             detailed modular Three.js geometry
└── render.js          Z-up camera, lighting, shadows, controls

server.mjs             local static server; optional in-process mock API
```

`server.mjs` explicitly rejects `--controller`, `--camera`, and `--allow-motion`. The deleted `js/live.js` path is no longer part of the unified application.

## 3. Coordinate system and dimensions

The model uses centimetres and a Z-up world. X runs across the board, Y runs from the rear toward the work area, and Z is height above the board/mat frame.

| Model check | Current value |
|---|---:|
| Board top to S2 | 9.6 cm |
| S2 to S3 | 10.5 cm |
| S3 to S4 | 12.6 cm |
| S4 to flange | 6.7 cm |
| Flange height, straight up | 39.4 cm |
| Pad gap, open / closed | 4.5 / 0 cm |
| Flange to pads, open / closed | 10.0 / 11.5 cm |

The detailed visual model includes servo cases, horns, paired metal brackets, fasteners, cable routing, base plates, mounting board/mat, camera, and the linkage-style claw. These details improve silhouette and clearance judgement, but collision geometry remains a deliberately simpler set of capsules and boxes.

The S1-to-mat offset is a photo estimate rather than a direct measurement. It must not be presented as validated.

## 4. Simulated controller behaviour

`controller.js` reproduces the useful behaviour of the team controller locally:

- servo targets are PCA9685 counts at a nominal 60 Hz;
- targets outside configured guards are rejected rather than clamped;
- only one simulated motion runs at once;
- motion uses a shared cubic smoothstep trajectory;
- HOME, PARK, jog, coordinated playback, stop, and re-arm are available;
- the red emergency button stops the simulation only.

The UI has two playback styles. Coordinated demo playback moves joints together. Controller-style preview decomposes a pose into one-joint moves and chooses a collision-clear order. Neither mode issues HTTP requests to a robot.

## 5. Collision, physics, and grasping

Arm parts use capsules and oriented boxes for self, board, and cube collision. Adjacent links are excluded where appropriate. New self/board contacts can stop simulated motion when “Stop on collision” is selected.

The mounting fixture is represented by compound collision shapes: lower and upper plates plus front and rear vertical supports. This preserves the real open cavity between the plates instead of treating the entire fixture envelope as a solid obstruction. With the current provisional, photo-aligned calibration, the longest claw finger reaches the 2.5 cm upper-plate height at PARK without penetrating it. No PARK-specific collision exception is used.

Cube dynamics are implemented locally with SAT/face-clipped contacts, sequential impulses, friction, gravity, rotational state, and sleeping. The kinematic arm contributes surface velocity, so it can push or topple cubes.

Grasping is intentionally rule-based. When the closing pads envelop a suitably placed cube, the cube is aligned, centred, and attached. Opening releases it with the hand velocity. This gives stable manipulation without claiming a high-fidelity friction model.

## 6. Planning and verification

Natural task phrases such as `blue on orange`, `grün auf blau`, and `yellow to 0 14` become a sequence of count targets:

1. Solve a top-down IK target with the gripper oriented for the cube.
2. Add vertical approach, closure, lift, transit, lower, release, and withdrawal poses.
3. Insert raised via poses when a direct joint-space transit intersects a stack.
4. Run the plan in a copied world with physics.
5. Report contacts, duration, grasp events, and final placement error.

This planner is intended to avoid obviously impossible or colliding simulated paths. Because calibration and envelopes are not fully measured, it is not a certified motion-safety system.

## 7. Camera and reference images

The camera tab renders the scene through the OAK camera’s fitted pinhole model. A repository image or user-selected local image can be overlaid for comparison.

Calibration uses known mat-marker corners:

1. normalised DLT homography;
2. focal length and initial pose from homography constraints;
3. Levenberg–Marquardt refinement of pose, focal length, and principal point.

Colour segmentation and back-projection detect cubes assumed to lie on the mat and can place those detections into the simulated scene. There is no live snapshot, MJPEG stream, camera proxy, or remote fetch in the unified build.

## 8. Interface

The dark responsive UI keeps the controller’s familiar concepts without copying its hardware connection:

- count sliders and numeric fields for S1–S6;
- HOME, PARK, jog, stop, re-arm, and limit/calibration concepts;
- planning, simulation playback, path and collision visualisation;
- cube scene editing and physics demo;
- simulated/reference camera tools;
- resizable desktop sidebar and stacked tablet/mobile layout;
- a persistent offline-twin badge and simulation-only notice.

The displayed application and document title remain exactly `PX-robo-arm-simulator`.

## 9. Validation

Run from `threejs-sim/`:

```sh
npm test
```

The 23 automated tests cover kinematics, controller emulation, collision, cube physics, planning, camera calibration, PARK alignment with the raised fixture plate, and direct HOME→PARK and PARK→HOME transitions. Browser changes additionally require a console check and manual exercise of the affected controls.

The transition tests establish only that the current digital twin encounters no modelled contacts and that the PARK claw endpoint aligns with the modelled upper plate. PARK remains an observed commissioning reference pose, not a physically validated safe transition.

## 10. Remaining measurement work

For dependable collision avoidance rather than dynamic realism, the highest-value physical data is:

1. calibrated HOME angle/direction and counts-per-degree for S1–S5;
2. photographed/measured joint centres in several known count poses;
3. maximum body envelopes for brackets, horns, servo cases, bolts, and cables;
4. gripper pad position and envelope across several S6 counts;
5. board, fixture, controller box, and cable obstacle locations in one common frame;
6. repeatable PARK clearance measurements and confirmation that the observed path is collision-free.

Until these are measured, the application should label fitted values as provisional and treat collision results as useful estimates rather than physical guarantees.
