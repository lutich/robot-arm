# px-robotoc-arm-sim

MuJoCo sandbox for the six-servo desktop arm, driven through the same control API as the
real arm. The physics sits behind robot-arm's `ServoDriver` port, so robot-arm's `Session`,
its HTTP API, its web page and its demo playback run unchanged against the simulation.

Design: [docs/simulation-concept.md](docs/simulation-concept.md).
Measurements: [docs/arm-dimensions.md](docs/arm-dimensions.md), [docs/servo-calibration.md](docs/servo-calibration.md).

## Setup

Needs Python 3.12+, [uv](https://docs.astral.sh/uv/) and a checkout of the robot-arm repo next
to this one (or `ROBOT_ARM_PATH` pointing at it). Tested against robot-arm commit `3792d64`.

```sh
uv venv --python 3.13 .venv
uv pip install -p .venv/bin/python -e ".[dev]"
ln -sf ~/.local/share/uv/python/cpython-3.13.0-macos-aarch64-none/lib/libpython3.13.dylib .venv/lib/   # lets mjpython (viewer) start with uv's Python
.venv/bin/python -m pytest          # about two minutes; the HTTP tests run a real server
```

## Run

```sh
.venv/bin/python -m armsim.server                 # sandbox API on http://127.0.0.1:8765
.venv/bin/mjpython -m armsim.server --viewer      # same, with the MuJoCo viewer window (macOS needs mjpython)
.venv/bin/python -m armsim.cli                    # text interface against the sandbox
.venv/bin/python -m armsim.cli --target http://raspberrypi.local:8765   # the same shell against the real arm
.venv/bin/python -m armsim.cli scripts/pick_red.txt                     # scripted session
```

With the server running, robot-arm's own page at http://127.0.0.1:8765/ and its 3D view at
`/simulation` show and drive the sandbox, Swagger is at `/docs`, and the rendered camera is
served at `/api/camera/snapshot.jpg` like on the Pi. Sandbox-only routes: `/sim/blocks`,
`/sim/state`, `/sim/reset`, `/sim/snapshot.png?camera=oak|top|side|front`, and
`POST /api/move_pose` (six counts plus an optional speed), the action proposed to robot-arm.

Text interface commands: `help` lists them. The important ones are `power on`, `move S2 450`,
`pose <six counts>`, `home`, `pick red`, `place T2`, `run red T1`, `demo import FILE`,
`demo load NAME`, `demo run`, `snapshot oak`, `log FILE`, `replay FILE`.

## The real arm

The real arm runs the robot-arm app on the Pi, currently at `http://10.102.4.103:8765` (Swagger at
`/docs`). Its API and the sandbox API are the same 35 routes with identical schemas; the sandbox only
adds `/api/move_pose` and the `/sim/...` routes. Every client here takes the target as an address:

```sh
.venv/bin/python -m armsim.cli --target http://10.102.4.103:8765          # text interface, real arm
.venv/bin/python -m armsim.web --api http://10.102.4.103:8765             # 3D view following the real arm: http://127.0.0.1:8780/arm3d
.venv/bin/python -m armsim.cli --target http://10.102.4.103:8765 scripts/stack_yellow_on_red.txt
```

Differences the tools handle for you:

- **Power on** on a hardware target asks the operator to confirm that the arm is physically in PARK,
  supported, and the servo power switch within reach; in scripts write `power on confirmed`.
- **No `move_pose` on the Pi:** the client falls back to single-joint moves in a safe order (slower).
- **No block positions on the Pi:** type them from the printed grid, `blocks set red 0.110 0.110`,
  before `pick`, `place` or `stack`. The task layer keeps them up to date after each set-down.
- **No cross-origin requests:** the robot-arm API refuses them, so the 3D view reaches the real arm
  through the proxy of `armsim.web` rather than with `?api=`.

Everything above the driver is the same code for both targets, so a pose log or demo that ran in the
sandbox is sent to the real arm count for count. Watch the first runs with the power switch in hand.

## 3D view (three.js, no other dependency)

`web/arm3d.html` draws the arm the way the kit is built, from the photos in `docs/images` and
the Amazon listing: 2 mm anodised sheet brackets with real hole patterns and rounded ends, the
two base plates, MG996R servos with horns, ears and pigtails, the PCA9685 board in pink foam,
zip-tied servo cables, the aluminium forearm tube with flanges, the hanging wrist bracket and
the gear-driven claw with rubber pads. Lit by a room environment with soft shadows. It follows whatever API serves it and otherwise runs on its own with sliders.

```sh
.venv/bin/python -m armsim.web                     # standalone: http://127.0.0.1:8780/arm3d.html
```

- Against the sandbox: http://127.0.0.1:8766/arm3d (blocks come from the MuJoCo physics).
- Standalone, or against the Pi: `arm3d.html?api=http://raspberrypi.local:8765`. Without a
  reachable API the page shows sliders, and blocks use a minimal built-in physics (gravity,
  stacking, pushing, grasping) written in `web/arm3d-core.js`.
- Cameras: Photo (like the product photo), Iso, Front, Top, Oak (the fitted real camera), Gripper.
- Connected to an arm, the sliders and HOME/PARK/Open/Close send the API's own actions; Power on
  and STOP are there too. The sequence row lists the arm's saved demos and runs them.
- Virtual only is the default: every control stays on the 3D model and nothing is sent to the arm.
  The toggle switches to LIVE (red) and asks for confirmation on a hardware target; it is not
  remembered, so each reload starts virtual.
  Sliders, HOME/PARK, Open/Close, sequences and Go & lower then animate the model (Go & lower
  asks the server for the poses with `dry: true`).
- Go to X, Y: type the table coordinates in mm (or press "Pick on table" and click), then
  "Go & lower" moves the open gripper above the point and lowers it to Z; "Go above" stops at
  safe height. Served by `POST /move_to` on the sandbox and by the proxy for the real arm
  (`armsim/mover.py`: inverse kinematics, reach check, then `move_pose` steps).
- URL parameters: `?pose=home`, `?cam=iso`.

Only three.js is vendored (`web/vendor`, r170). The numbers come from `web/arm3d-config.js`,
generated from the YAML configs with `python -m armsim.export_web_config`; a test keeps it in sync,
and `node --test tests/arm3d-core.test.js` checks the JavaScript core.

## Layout

| Path | Content |
|------|---------|
| `config/robot.yaml` | Link lengths, gripper linkage, servo model, count-to-angle calibration |
| `config/scene.yaml` | Table frame, base position, blocks, slots, physics, cameras |
| `armsim/model_builder.py` | MuJoCo model generated from the two configs |
| `armsim/driver.py` | `MujocoDriver`, robot-arm's `ServoDriver` port over the physics |
| `armsim/session.py` | robot-arm `Session` plus `move_pose` |
| `armsim/server.py` | Sandbox launcher: Session, HTTP server, camera service, `/sim` routes |
| `armsim/client.py` | `HttpArmClient` and `SessionArmClient`, one interface for sandbox and real arm |
| `armsim/kinematics.py`, `armsim/tasks.py` | Closed-form IK and the pick-and-place sequence in counts |
| `armsim/replay.py` | Import of Pi demo records, pose-log replay |
| `armsim/cameras.py` | Named cameras, fit of the real camera from robot-arm's calibration, `CameraSource` |
| `armsim/cli.py` | Text interface |
| `armsim/web.py`, `web/` | Stdlib static server and the three.js 3D view |
| `tests/` | Calibration, model geometry, driver contract, kinematics, grasp physics, task, replay, cameras, HTTP end to end |

## Assumptions still to verify on the real arm

All in `config/robot.yaml` and `config/scene.yaml`, each a single number:

- Joint angles at the HOME counts and the direction of rising counts per servo (taken from
  robot-arm's browser model; S4 assumed 105° so that PARK rests on the base plate).
- The gripper jaws open across the arm plane at S5 = 480 (`gripper.pads_axis: y`, confirmed on the real arm), and 650 counts is fully open (still to confirm).
- The S1 axis stands 40 mm inside the near table edge, centred.
- Block mass 12 g, pad friction 1.0.
