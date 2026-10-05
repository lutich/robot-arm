# Commissioning backend and browser API

The local app uses one Python backend and a plain HTML/CSS/JavaScript frontend.
There are no new runtime dependencies. Paths below are under
`src/roboter_arm/control/`; `scripts/manual_control.py` launches the app.
`presentation/http_api.py` contains the HTTP adapter.
`application/session.py` contains the authoritative `Session` application service;
`infrastructure/drivers.py` supplies the explicit preview/hardware adapters. The
service has no dependency on HTTP. Its commands use the existing
`domain/motion.py` trajectories and `application/motion_runner.py` ownership;
`infrastructure/pose_store.py` and `infrastructure/demo_store.py` store PARK/HOME
definitions and immutable Demo records without controller access; `session_for` in
`presentation/http_api.py` wires them into the session.

The browser files are `web/manual_control.html`, `web/manual_control.css`
and `web/manual_control.js`. They render backend state and submit actions.
The browser does not interpolate trajectories, execute a sequence itself or
decide whether a requested movement is allowed.

## Access and status

The server binds to `127.0.0.1` by default. The Pi service binds every interface
(`--host 0.0.0.0`), so any device on the Pi's Wi-Fi or Ethernet network can use it.
There is no authentication; run it only on a network you control. Every request
needs a `Host` that is an IP address, a name without dots or a `.local` name, and
every POST needs `Origin` equal to `http://<Host>` and `Content-Type: application/json`.
This keeps other websites open in the same browser from sending actions. Requests and
responses are JSON. Success is `{"ok": true, "result": ...}`; rejected requests return
an error status (see below) with `{"error": ...}`.
An accepted motion starts a backend worker; HTTP success means accepted, not
that the arm reached the target. Poll status to observe completion or failure.

`GET /api/state` is read-only. It reports commanded counts, effective and outer
joint bounds, motion phase, armed/busy/output-off state, errors and Demo progress.
It does not initialize hardware, measure joint position or refresh readiness.
There is no browser heartbeat. Closing, hiding or losing the browser does not
stop the arm: it keeps holding, and a running move or Demo continues. Use Stop or
Power off, or the physical servo power switch.

## Actions

All paths below start with `/api/`. Joint channels are integers 0–5 and targets
are integer PCA9685 counts at nominal 60 Hz. Counts are not degrees.

| Path | JSON body | Behavior |
| --- | --- | --- |
| `power_on` | `{"park_confirmed": true}` | Requires fresh physical supported PARK/readiness; enables PARK then moves to HOME. |
| `power_off` | `{}` | Returns known commands to PARK, then disables all outputs. |
| `stop` | `{}` | Cancels immediately and directly disables; no recovery movement. |
| `move` | `{"channel": 2, "target": 410, "first_clear": false}` | Moves one joint; others hold. |
| `jog` | `{"channel": 2, "direction": 1}` | Adds (+1) or subtracts (-1) the configured step from the held command, using existing movement guards. |
| `settings` | `{"step_size": 10, "demo_speed": 5}` | Atomically changes both integer settings while idle; no movement. |
| `go_pose` | `{"name": "HOME"}` or `{"name": "PARK"}` | Moves all six and retains holding. |
| `limits` | `{"channel": 2, "low": 390, "high": 600}` | Changes session bounds without moving; retains current/PARK/HOME targets and existing outer guards. |
| `save` | `{"name": "HOME", "observed": true, "provenance": "..."}` (or `"name": "PARK"`) | Archives full-pose commands and makes them the active definition, retained after restart; no movement. |
| `prepare` | `{"supported": true, "power_off": true}` | Older commissioning flow: acquires/prepares the controller and verifies outputs off; requires current physical support and servo power OFF. |
| `arm` | `{"ready": true, "supported": true, "switch_ready": true}` | Older commissioning flow: requires prepared/off outputs and fresh attended powered readiness; sends no positioning command. |
| `demo/name` | `{"name": "My demo"}` | Names the draft. |
| `demo/add` | `{}` or `{"name": "Position 1"}` | Captures all six settled commands. |
| `demo/replace` | `{"index": 0}` | Replaces a position with current settled commands. |
| `demo/rename` | `{"index": 0, "name": "Ready"}` | Renames a position. |
| `demo/reorder` | `{"index": 1, "direction": -1}` | Moves a position up (-1) or down (+1). |
| `demo/remove` | `{"index": 0}` | Removes a position from the draft; saved files stay intact. |
| `demo/next` | `{}` | Executes the next position, then holds. |
| `demo/run` | `{}` | Starts at position 1 and runs through once, then holds. |
| `demo/pause` | `{}` | Finishes the current step, then holds. |
| `demo/resume` | `{}` | Continues the remaining steps after a pause. |
| `demo/restart` | `{}` | Resets progress without moving. |
| `demo/save` | `{}` | Writes a new immutable scoped JSON file; returns its filename. |
| `demo/load` | `{"filename": "<32 hex characters>.json"}` | Loads a saved draft with progress reset; does not move. |

`prepare` and `arm` are for bounded tests on a restricted session: a subset of
joints (`manual_control.py --channels`) or ranges that leave PARK or HOME out,
where `power_on` refuses. They arm without the automatic PARK/HOME movement, the
page does not use them, and they do not bypass their existing current
physical-readiness requirements.
When commands are unknown in that flow, the first `move` needs
`first_clear: true` for its explicit initial target. In hardware mode that
acknowledgment must come from the attending operator.

`state.settings` reports `step_size` (default 10 positive integer counts),
`demo_speed` (default 20 integer counts/s) and `demo_speed_max` (100). Demo speed
may be set from 1 to 100 and applies to Next, Run and Resume, including deadline
preflight and per-step timeout calculations. Manual jog/moves and named
PARK/HOME transitions retain their existing 20-counts/s commanded speed.
Settings apply to the running app; saving/loading a Demo retains its positions
without replacing these settings. Arrow signs refer to command counts, not
verified physical directions. Unknown commands or targets beyond current
bounds are rejected without clamping or an initial positioning move.

## HOME and PARK persistence

Save requires all six settled commands and the existing observation confirmation.
`state.joints[].home` and `.park`, Go HOME/PARK, Power on and Power off use the
active definitions. Saving updates the running session after the definition is
written successfully; it does not send servo commands.

Active definitions are `artifacts/poses/synthetic/HOME.json` and `PARK.json` for
preview, or `artifacts/poses/commissioning/HOME.json` and `PARK.json` for hardware.
Each save also retains its immutable UUID-named observation record. These files
are local to the machine running the app and are excluded from deployment.
Scope, units and six integer commands inside historical bounds are checked at
startup. A missing definition uses the shipped reference default; an invalid
definition rejects startup. Applied session bounds still constrain all motion.

Older archive-only saves are not activated automatically. Re-save the desired
pose with the updated app. Loading definitions at startup does not initialize
hardware or establish the arm's physical position.

## Demo ownership and persistence

`state.demo` contains `name`, `positions`, `cursor`, `status`, `mode`, `pause_requested`
and `saved`. Each position contains a name, six command counts and current
validity/error. `cursor` is the number of successfully completed positions.
Status is idle, running, pausing, paused or complete; mode distinguishes a
single step from running all positions. Invalid stored positions
remain visible and cannot be played. `saved` lists scoped filenames, names and
position counts for the Load dialog.

Capture/replacement requires armed, idle operation with all commands known.
Playback validates the requested targets before starting and rejects overlapping
motion. Editing, limits changes and manual repositioning reset progress.
Pause occurs at a step boundary; EMERGENCY STOP, browser loss and faults cancel
future steps, clear starting commands and require fresh initialization.
Save/Load never stores or restores an execution cursor.

Files are under `artifacts/demos/synthetic/` for preview or
`artifacts/demos/commissioning/` for hardware. Scope, units and nominal frequency
are checked on load. Preview records cannot be loaded into hardware mode.
Saved counts have no position feedback and do not establish calibrated travel
or collision clearance. Min/Max edits apply only to this running session.

## Using the API

Open the frontend through the same URL as the API, for example
http://raspberrypi.local:8765/. Browsers send the matching `Origin` on POST;
other clients must send it themselves, with `Content-Type: application/json`.
POST bodies are JSON (including `{}` for actions without arguments), at most
8192 bytes. GETs need only an allowed Host.

The following Python example reads status only:

```python
import json
from urllib.request import urlopen

with urlopen("http://raspberrypi.local:8765/api/state", timeout=5) as response:
    state = json.load(response)
print(state["mode"], state["armed"], state["busy"])
```

An action from the command line; `demo/restart` resets Demo progress and does
not move:

```sh
curl -X POST http://raspberrypi.local:8765/api/demo/restart \
     -H 'Origin: http://raspberrypi.local:8765' -H 'Content-Type: application/json' -d '{}'
```

## Interactive docs and request types

`/docs` serves Swagger UI and `/openapi.json` the OpenAPI spec, both generated
from the running code. Swagger groups the endpoints as State, Power, Motion,
Poses, Session settings, Demo and Bounded tests. Swagger UI loads its scripts from a CDN
(cdn.jsdelivr.net), so `/docs` needs internet on the viewing device;
`/openapi.json` works offline. "Try it out" sends real actions, including
`power_on` and `move` in hardware mode, without the page's readiness checkbox.

Request bodies are checked strictly before they reach the session: booleans must
be JSON `true`/`false` and integers JSON integers (`"4"`, `4.0` and `true` are
refused), and unknown fields are refused. Such a refusal reads
`<field>: <reason>`. The session still enforces ranges and physical readiness.

| Status | When |
| --- | --- |
| 400 | Wrong type or unknown field, a request the session refuses, an empty or over-8192-byte body, or a Content-Type other than `application/json` |
| 403 | Host not allowed, or a POST without the matching `Origin` |
| 404 | Unknown path |
| 405 | Wrong method for a path, for example GET on an action |
| 500 | Driver or server failure; the session stops first |

Guard failures return 403. Invalid actions/arguments return 400. Driver/server
failures attempt direct stop and return 500. Unknown GET paths return 404.
No action response proves physical arrival: wait until state is idle and
inspect `error`, `phase`, `commands` and Demo progress. Commands remain
commanded counts, not measured position.

This API retains the existing commissioning actions. Future clients can extend
application actions without duplicating motion or bypassing readiness. See
[architecture](architecture.md). No agent adapter or agent-specific endpoint is
implemented here.
