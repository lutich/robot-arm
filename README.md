# Raspberry Pi robot arm starter

Python control software and a browser app for an existing six-servo desktop
arm driven by a Raspberry Pi and PCA9685. This repository is a hackathon
starting point and a documented reference for people using similar hardware.

<p align="center">
  <a href="web/home-illustration.png"><img src="web/home-illustration.png" height="240" alt="Reference robot arm with its base, shoulder, elbow, wrist and gripper labeled"></a>
</p>

The reference arm illustration labels servos S1–S6, corresponding to software
channels 0–5.

## System connections

<p align="center">
  <a href="docs/images/system-connections.svg"><img src="docs/images/system-connections.svg" height="540" alt="Mac laptop, Raspberry Pi, PCA9685, robot arm and their data and power connections"></a>
</p>

The Pi runs the frontend and API as a service on its Wi-Fi and Ethernet network;
the Mac laptop deploys over SSH and opens the app in a browser.
The Pi controls the arm through the PCA9685; the servos have a separate power
supply. The owner confirmed this setup and its wiring on 2026-10-04; see
[hardware wiring](docs/hardware.md#power-and-wiring) for connection details.

## Wiring

<p align="center">
  <a href="docs/images/wiring.svg"><img src="docs/images/wiring.svg" height="720" alt="Confirmed wiring: Mac laptop, Pi header pins, PCA9685, separate servo power supply and three-wire connections to servos S1–S6"></a>
</p>

Click an image for the full-size version. The wiring diagram shows the confirmed
connections and physical Pi pin numbers. Component illustrations and pin
placement are schematic. See [hardware wiring](docs/hardware.md#power-and-wiring)
for the pin table and diagram regeneration command.

Start with the [laptop preview](docs/quickstart.md), then follow
[Pi preparation](docs/raspberry-pi.md) and [deployment](docs/deployment.md).
Read [hardware](docs/hardware.md) and [operations](docs/operations.md) before
powered operation.

## Run without hardware

Use `uv` and Python 3.12+ on macOS or Linux. Preview and tests use the locked
FastAPI and Uvicorn packages that `uv sync` installs; there is no frontend build step. Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then create the locked project environment:

```sh
uv sync --locked
uv run --locked python scripts/manual_control.py
```

Open **http://127.0.0.1:8765/**. Read-only API state is at
**http://127.0.0.1:8765/api/state**. Preview simulates commands and writes
synthetic records; it is not a physics simulator.

### 3D arm viewer

Section 03 of the page, below Joint control, draws the arm in 3D (`web/sim_arm_view`).
It is a view only: everything it shows comes from the API of the *mock controller*, which
is `manual_control.py` without `--hardware`. The mock runs the arm model in
`src/roboter_arm/sim` behind its servo driver and reports it on
`GET /api/sim/model` (dimensions, servo calibration) and `GET /api/sim/world`
(pose, contacts, events). A controller started with `--hardware` simulates
nothing and has no `/api/sim/*`.

- **Preview** (`manual_control.py`): the simulation is the synthetic arm. A
  command that would drive it into the board or into itself is refused before
  it is applied, and the session stops with outputs off
  (`Move stopped: SimulatedCollision`). Fingers pressing up to 2 cm onto the
  board only count as *pressing*: a real servo stops short of its command there.
- **Following the real arm** (`manual_control.py --follow http://<pi>:8765`):
  open the mock's page, http://127.0.0.1:8765/. Every `/api/*` request from
  the page goes on to the real controller, so this page controls the real arm.
  The mock reads the real `/api/state` ten times a second and its simulation
  follows those commands. Contacts are only shown, never stop the real arm,
  because the servo calibration (`q_ref`, `direction` in
  `src/roboter_arm/sim/domain/arm_model.json`) is not measured.

```sh
uv run --locked python scripts/manual_control.py --follow http://raspberrypi.local:8765
```

The model can hold cubes (grasp, carry, release onto a stack, no physics), but
nothing places them yet. The section also holds the camera tools: simulated OAK
view, calibration from the mat markers and cube recognition. They read
`/api/camera/*`, which the mock forwards when following, and fall back to a
saved frame without a camera. The viewer's mat, camera and base fixture come from
`web/sim_arm_view/view.json`.

### Trajectory planning

Section 04 plans a task and rehearses it, also on the mock controller only. Type
`blue on orange`, `green to 5 15` (board cm), `grab red`, add `home` to end at HOME,
choose the start pose (PARK, HOME or the current commands) and press **Plan & rehearse**.
The mock plans top-down pick-and-place waypoints, splits them into the steps the
controller executes, rehearses them in its own copy of the cube scene and answers with a
verdict, events, contacts and a timeline that the section plays back in its own 3D view.
Nothing is sent to the arm, and the arm simulation of section 03 is not touched.

The planner lives in `src/roboter_arm/sim/domain/planning`. Its parameters (single-servo or
coordinated steps, speed, hover heights, gripper hold count, acceptance tolerances) and the
default cube scene are in `src/roboter_arm/sim/domain/planning/planning.json`, documented in
`parameters.py` there. The mock reads the file on every request, so edits apply without a
restart. To try values without editing it, change them in the section's Parameters and Scene
panels; they go with your next plans and stay in your browser. **Copy as planning.json**
gives the text to paste into the file. Contacts in the rehearsal are reported, not refused,
unless `acceptance.fail_on_contact` is on, because the servo calibration is not measured.

## Connect and deploy

Configure `.pi.json` from `pi.example.json`. Provision SSH authentication and
verify the Pi's host fingerprint as described in the deployment guide.

```sh
uv run --locked python scripts/setup_connection.py --fingerprint 'SHA256:<verified fingerprint>'
uv run --locked python scripts/deploy.py
uv run --locked python scripts/setup_pi.py --install-system
uv run --locked python scripts/service.py install
```

`service.py install` runs the hardware app as a Pi service that also starts at
boot. Open **http://raspberrypi.local:8765/** from any device on the Pi's network.
Setup, deployment and app launch do not send servo commands; hardware is
initialized only by an attended Power on in the UI. Later deploys restart the app
and refuse while the arm is powered.

## What is available

- Manual joint movement, count jogs, PARK/HOME controls and Demo recording/playback.
- One application owner for motion, command validation and cancellation.
- A FastAPI HTTP API with JSON actions and Swagger docs at `/docs`, open to the local network; POSTs must come from the same-origin page.
- Configurable SSH key/password access, conflict-checked deployment and passive checks.
- A bundled, hash-locked Pi runtime for Linux aarch64 / CPython 3.13.

Observed PARK/HOME commands and historical joint ranges belong to the reference
arm. They do **not** establish measured position, validated travel, collision
clearance or full-arm supply capacity. Calibrated XYZ control and agent
implementations are outside this starter's scope.

## Repository map

| Path | Purpose |
| --- | --- |
| `scripts/` | Command entry points; thin launchers into `src/` |
| `src/roboter_arm/control/` | Session lifecycle, motion, HTTP API, drivers and commissioning checks |
| `src/roboter_arm/calibration/` | Calibration evidence rules and recording |
| `src/roboter_arm/provisioning/` | Laptop-side Pi connection, deployment, inspection and Wi-Fi tooling |
| `src/roboter_arm/shared/` | Joint catalog and repository root used by all contexts |
| `web/` | Existing HTML/CSS/JavaScript and reference illustrations |
| `config/reference-arm/` | Selected reference settings and observations |
| `pyproject.toml`, `uv.lock` | uv project metadata and laptop environment lock |
| `requirements/`, `runtime/` | Tested Pi dependency lock and matching wheels |
| `tests/` | Hardware-free control, HTTP and deployment tests |
| `docs/api.md` | Routes, request bodies, responses and extension boundary |
| `docs/architecture.md` | Responsibilities and dependency direction |

```sh
uv run --locked python -m unittest discover -s tests -p 'test_*.py'
```

The tests use fake drivers and temporary files; HTTP tests bind localhost.
See [contributing](CONTRIBUTING.md) for development and physical validation.
