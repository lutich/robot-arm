# Notes for agents working in this repo

Start with `HANDOVER.md` (state and open items) and `threejs-sim/README.md` (the simulator).

## Safety

- **Never command the real arm** (controller at http://10.102.4.103:8765) unless the user says so in that moment. Keep even reads (state, camera snapshots, video) to a minimum: other people use the arm, and the camera and servos can overheat.
- Test against the shared simulation: `node threejs-sim/server.mjs` (the API in `docs/arm-api.md`; it starts on the simulated arm). Never switch it to the real arm (`POST /api/backend {"target": "real"}`) without the user's go-ahead.
- The simulator server is read only towards a real controller unless it is started with `--allow-motion`.

## Checks

```powershell
node --test threejs-sim\test\*.test.mjs
```

## Comparing the model with photos

Full procedure and the latest results: `docs/model-photo-comparison.md`. In short:

1. Add the photo to `docs/images/photos/` and a view with a rough camera to `threejs-sim/tools/photo-views.json`.
2. `node tools/compare.mjs --only <name> --keypoints` (in `threejs-sim/`) shows the model's named points.
3. Mark the same points in the photo (`"points"`, photo pixels). Prefer mat marker corners and joint axis points; leave out points on parts the model draws wrong.
4. `node tools/fit-views.mjs --only <name>` fits camera and joint angles; `--joint` fits all photos of one pose together; `--save` writes the result.
5. `node tools/compare.mjs` renders model | photo | overlay into `docs/images/compare/`.
6. Record match / partial / no match and the model differences in `docs/model-photo-comparison.md`. After changing the model, keep `js/collision.js`, `js/model/cables.js` and the keypoints in `js/viewfit.js` in step, then refit and compare the RMS with the table there.
