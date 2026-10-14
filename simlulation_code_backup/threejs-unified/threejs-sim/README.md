# PX-robo-arm-simulator

A polished, responsive, dark browser digital twin for the PX six-servo arm. Three.js renders the scene; plain JavaScript provides kinematics, simulated controller behaviour, collision checks, cube physics, task planning, camera calibration, and cube recognition.

This build is intentionally **simulation-only**. It has no Raspberry Pi, GPIO, PCA9685, serial, WebSocket, or robot-control connection. The user interface follows the concepts and servo-count vocabulary of the `lutich/robot-arm` controller, but every control acts only on the in-browser model.

Three.js r170 is vendored in `vendor/`, so there are no npm dependencies. Node.js is needed only for the static development server and tests.

![Simulator](../docs/images/threejs-sim.png)

## Run

```sh
cd threejs-sim
npm start
```

Open <http://127.0.0.1:8770/>. Use `--port` and `--host` after `node server.mjs` to change the local address. Opening `index.html` directly does not work because browsers block ES modules from `file://`.

For the optional local mock API and tests:

```sh
npm run mock
npm test
```

The server rejects the former `--controller`, `--camera`, and `--allow-motion` options. It cannot proxy requests to robot hardware.

## What it does

| Area | Behaviour |
|---|---|
| Model | Detailed brackets, servos, horns, cables, base, board, mat, claw gripper, camera, and fixtures based on the supplied dimensions and reference photos |
| Kinematics | S1–S5 count/angle conversion, forward kinematics, top-down inverse kinematics, and an S6 linkage approximation |
| Controller model | PCA9685 counts at 60 Hz, guards, rejection instead of clamping, one simulated motion at a time, cubic smoothstep, HOME/PARK, jog, stop, and re-arm semantics |
| Collision | Capsules and boxes for the arm; compound lower/upper fixture plates with an open cavity; self, board, fixture, and cube contacts; optional collision-shape display and stop-on-contact |
| Cube physics | Local rigid bodies with rotation, gravity, contact impulses, friction, stacking, toppling, pushing, and sleeping |
| Grasping | Claw closure detects, aligns, holds, carries, and releases suitable cubes |
| Planning | Commands such as `blue on orange`, `grün auf blau`, and `yellow to 0 14`; vertical approach/lift, stack-aware transit points, and headless verification before playback |
| Playback | Coordinated demo playback or a controller-style one-joint-at-a-time preview, both entirely simulated |
| Camera tab | Simulated OAK view, saved/local reference-image overlay, marker calibration, and coloured-cube recognition without a live camera connection |
| UI | Dark responsive layout, resizable desktop sidebar, tablet/mobile stacking, joint sliders and count fields, controller-style actions, logs, scene editor, and emergency simulation stop |

## Safety boundary

- No browser module connects to the physical controller.
- `server.mjs` serves local files and can optionally expose an in-process mock API only.
- Camera input is limited to a repository image or an image chosen from local disk.
- The emergency-stop button stops simulated motion and clears the simulated motion queue; it does not claim to stop hardware.
- Historical controller values and fitted calibration are modelling inputs, not validated safe physical limits.

## Files

| File | Content |
|---|---|
| `js/config.js` | Dimensions, servo table, provisional calibration, controller rules, mat, and camera |
| `js/kinematics.js` | FK, top-down IK, counts/angles, and gripper linkage |
| `js/controller.js` | In-browser controller emulation; also used by the optional local mock |
| `js/collision.js` | Distance/SAT tests, arm collision shapes, and contact reporting |
| `js/physics.js` | Rigid-body contacts, impulses, friction, and sleeping |
| `js/world.js` | Arm pose, grasp logic, physics stepping, and collision state |
| `js/planner.js` | Pick/place/stack plans, transit points, verification, and single-joint previews |
| `js/calib.js`, `js/aruco.js`, `js/detect.js` | Reference-image camera calibration and cube recognition |
| `js/model/` | Modular detailed Three.js geometry and materials |
| `js/render.js` | Z-up renderer, lighting, environment, and camera controls |
| `js/camview.js` | Simulated camera view and local reference-image tools |
| `js/demo.js` | Tower-building physics demonstration |
| `js/app.js` | UI wiring and animation loop |
| `server.mjs` | Local static server and optional in-process mock API |

## Validation status

`npm test` runs 23 automated tests covering kinematics, controller emulation, collision, HOME/PARK transition verification, physics, planning, and camera calibration.

The HOME and PARK poses and both direct HOME↔PARK transitions now pass the simulator's collision checks. At PARK, the longest claw finger aligns with the top of the 2.5 cm raised fixture plate without penetrating it. The fixture is modelled as separate lower and upper plates with front and rear supports, so its open cavity is no longer treated as a solid obstruction. No collision is suppressed or reclassified. This is simulator verification only; it does not promote the observed PARK pose or transition to validated physical-arm motion.

The link measurements match the current schematic values, but the joint angle at HOME, joint directions, some body envelopes, and the S1-to-mat offset still need physical measurement. The current values remain provisional/fitted and can be edited in the simulator calibration panel. Collision avoidance is therefore a digital-twin estimate, not a guarantee of real-arm safety.
