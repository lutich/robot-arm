# PX robot arm simulator — Three.js branch

This is the `feat/threejs-vk` browser implementation of the PX robot-arm
simulator. It is independent of the team's Blender, MuJoCo and Unreal work.

The application currently supports calibration-focused virtual simulation:

- a detailed photo-informed Three.js arm, calibration mat, desk fixture,
  cables, servos, horns and linkage claw;
- a versioned millimetre `arm-spec.json` with explicit provenance;
- modular forward/inverse kinematics;
- direct joint-space interpolation and numerical IK;
- text pick/place tasks with approach/lift waypoints and physics outcome checks;
- coordinated and one-servo-at-a-time playback;
- lazy-loaded Rapier3D contact physics and dynamic solid cubes;
- simulated cube interaction without motion-gating collision checks;
- calibrated gripper-marker tracking for planned/simulated/observed TCP comparison; and
- simulation plus explicitly guarded read-only/disarmed/armed operational
  modes for the existing robot-arm API.

The provisional collision planner and runtime collision stops were removed for
the current calibration phase because their unmeasured envelopes produced
false rejections. Automated physical task/pose playback is disabled. Explicitly
armed single-joint slider commands still retain arming, bounds, rate limiting,
backend confirmation and emergency stop.

Simulation is the default and startup performs no physical-arm request. The
hardware has no position feedback, so operational state is labelled commanded
pose. Historical bounds are not described as safe limits, and PARK remains an
observed supported commissioning reference rather than a motion-validated
pose.

## Run and verify

```sh
cd web
npm install
npm test
npm run dev
```

Open the URL printed by Vite. Build the production site with:

```sh
npm run build
```

For a same-origin, read-only robot connection, build first and use the guarded
gateway. Motion forwarding needs the separate explicit flag:

```sh
npm run serve -- --controller http://ROBOT_HOST:8765
npm run serve -- --controller http://ROBOT_HOST:8765 --allow-motion
```

The build succeeds with a non-fatal large-chunk warning for Three.js and the
separate embedded Rapier WebAssembly runtime.

## Key files

| Path | Purpose |
| --- | --- |
| `web/js/app.js` | active preserved UI, camera/model orchestration and integration |
| `web/js/gripper-tracker.js` | marker pose and physical TCP estimation |
| `web/js/model/` | detailed photo-informed robot, fixture, mat, cables and gripper |
| `web/js/advanced.js` | canonical-mm adapter for the preserved visual calibration |
| `web/src/core/` | pure kinematics, calibration planning, playback and physics adapters |
| `web/src/operational/client.js` | guarded same-origin robot API state machine |
| `web/src/operational/gateway.js` | allowlist policy for proxied robot routes |
| `web/server.mjs` | production static server and read-only-by-default API gateway |
| `web/public/arm-spec.json` | millimetre model, planner settings and provenance |
| `web/test/` | 39 model, planner, physics, tracking, connection and reference-integrity tests |
| `web/README.md` | usage and operational-mode guide |
| `docs/threejs-implementation.md` | detailed design and validation status |
| `docs/arm-photo-reference.md` | hardware-photo index and overlay validation workflow |
| `docs/gripper-marker-tracking.md` | printable marker setup and TCP comparison procedure |

The root repository may also contain separate Blender work in other checkouts.
Do not combine it with this branch without an explicit specification-integration
decision.
