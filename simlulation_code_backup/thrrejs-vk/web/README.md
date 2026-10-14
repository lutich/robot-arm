# PX-robo-arm-simulator

Three.js browser digital twin for the PX arm. It uses Vite, plain JavaScript,
millimetre canonical data, numerical IK and Rapier3D cube dynamics. It does not
depend on Blender.

## Run

Node.js 20+ and a WebGL-capable browser are required.

```sh
npm install
npm test
npm run dev
```

Open the Vite URL. For the guarded gateway at `127.0.0.1:8770`:

```sh
npm run build
npm run serve
```

Use a controller only when explicitly testing Live Mode:

```sh
# Read-only state and camera; no ordinary motion forwarding
npm run serve -- --controller http://ROBOT_HOST:8765

# Attended single-joint calibration; motion routes enabled
npm run serve -- --controller http://ROBOT_HOST:8765 --allow-motion
```

## Current calibration mode

Implemented:

- detailed photo-informed arm, linkage claw, fixture, mat and cables;
- HOME, observed commissioning PARK and simulation-only STRAIGHT UP;
- S1–S6 sliders and reference-pose buttons;
- millimetre FK, numerical IK and direct waypoint interpolation;
- text pick/place waypoints and two simulation playback styles;
- Rapier board/fixture bodies, arm interaction bodies and dynamic cubes;
- read-only camera calibration, color-cube detection and marker/TCP tracking;
- recorded-pose mat map and servo-count table; and
- guarded simulation/read-only/disarmed/armed operating states.

Not active:

- geometric self/board/fixture/cube collision validation;
- clearance routing, RRT* search or collision-stop playback; and
- automated physical HOME/PARK or task playback.

The earlier provisional collision envelopes produced false path rejections
while camera/servo calibration is still being established, so those modules
and controls were removed. Rapier remains for simulated object dynamics only;
it does not approve or stop motion.

## Using the simulator

1. Use **HOME**, **PARK** or **STRAIGHT UP** only in Simulation mode.
2. Choose a task preset or enter `grab green cube` / `move orange to (80, 240) mm`.
3. Select **Plan only**. This solves IK and checks servo bounds, but does not
   check clearance.
4. Select **Run in sim** and inspect the animated result and cube outcome.
5. Use **Scan camera** to refresh the real camera observations.

Automated real playback is visibly disabled. In armed Live Mode, only an
explicit single-joint slider release can send a move. Keep jog steps small,
keep the workspace clear and remain at the physical power switch.

## Live Mode boundary

The browser never accesses GPIO/PCA9685 directly. The gateway exposes the
existing `lutich/robot-arm` API through a same-origin allowlist:

- `GET /api/state` for read-only commanded state;
- camera GET routes for observation;
- `POST /api/stop` for emergency stop;
- with `--allow-motion`: power and single-joint move routes only.

The client retains explicit arming, supported-PARK confirmation, per-session
backend bounds, command-rate limits, one-servo command shape, response
confirmation and stop-on-request-failure. The physical arm has no position
feedback, so counts are labelled **commanded pose**, never measured pose.

## Specification status

`public/arm-spec.json` schema version 2 uses millimetres. It separates measured
dimensions, provisional assumptions, observed commissioning poses and
historical unvalidated bounds. `world.physics.robotRadii` are physics-only
interaction approximations, not safety clearance.

HOME and PARK both retain S6=650. PARK is an observed supported commissioning
pose on the raised 25 mm fixture plate, not a motion-validated transition.

See [the implementation notes](../docs/threejs-implementation.md),
[photo references](../docs/arm-photo-reference.md), and
[gripper-marker guide](../docs/gripper-marker-tracking.md).
