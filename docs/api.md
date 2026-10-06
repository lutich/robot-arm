# Commissioning backend and browser API

The local app uses one Python backend and a plain HTML/CSS/JavaScript frontend.
Preview needs no camera dependency; OAK access uses the optional pinned
[camera runtime add-on](../runtime/README.md#optional-oak-camera-add-on). Paths below are under
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
action responses are JSON. Success is `{"ok": true, "result": ...}`; rejected requests return
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

## Optional camera access

The camera is disabled by default. Install the add-on explicitly on the Pi,
connect an OAK-D Lite through USB 3 (use a powered hub if needed), and opt in:

```sh
.venv-runtime/bin/python scripts/manual_control.py --camera oak --host 0.0.0.0
```

This command keeps the servo driver in preview. First check the camera with
servo power off. `--hardware --channels 0 1 2 3 4 5` separately permits attended
servo preparation through the existing controls. The Pi app service also needs
an explicit `service.py install --camera oak`; see [deployment](deployment.md).
Avoid running two app instances on the same port or camera.

The adapter supports OAK-D Lite on RVC2: RGB CAM_A / IMX214 (AF or fixed focus)
and stereo CAM_B/CAM_C / OV7251. The fixed combined profile is 1280 × 720 at
5 fps. RGB is cropped, undistorted NV12 encoded on the OAK as MJPEG at quality
85. The stereo pair uses native 640 × 480 inputs; depth is aligned to the
actual RGB output and resized to its pixel grid. This does not increase the
stereo sensors' resolution. Resolution, encoder and stereo settings are fixed.

| Route | Result |
| --- | --- |
| `GET /api/camera/status` | Camera availability, run ID, requested revision/groups, capabilities, latest frame readbacks/age, pending/locked state and last error. No camera or servo initialization. |
| `POST /api/camera/settings` | Queue supported changed groups; omitted groups stay unchanged. |
| `GET /api/camera/snapshot.jpg` | Fresh JPEG with exact matching metadata in the JSON `X-Camera-Frame` header. |
| `GET /api/camera/stream.mjpg` | MJPEG from the shared latest-frame cache. Each part has `Content-Length` and `X-Camera-Frame`. Slow viewers skip frames. |
| `GET /api/camera/capture` | One matched pair as JSON: `metadata`, base64 `rgb_jpeg`, `depth_png` and `depth_preview_png`. Depth fields are null for a source without depth. |
| `GET /api/camera/depth/snapshot.png` | Lossless 16-bit grayscale PNG: depth in millimetres, zero unknown. Matching `X-Camera-Frame` metadata. |
| `GET /api/camera/depth/preview.png` | Display-only RGB8 PNG: near red (200 mm), far blue (3000 mm), unknown black. Values outside that display range saturate; raw depth is preserved. |
| `GET /api/camera/depth/stream` | Multipart live PNG previews from the same pair cache, with matching part metadata. |

Status adds `depth_available`, `depth_profile` and `capabilities.has_depth`.
Pair metadata includes nested `depth` values: dimensions, `unit: "mm"`,
`invalid_value: 0`, `aligned_to: "rgb"`, source sequence, valid-pixel fraction,
acquisition time/age and measured `sync_delta_ms`. RGB metadata includes its
own source sequence and acquisition time. Pairs require device timestamps
within 20 ms; this is a measured time tolerance, not identical shutter timing.
Both members must be fresh and captured after a settings dispatch. The paired
OAK profile requires both outputs; a stereo failure makes camera output
unavailable while the arm Session remains independent.

Capture in the browser retains RGB, raw depth and its preview from one response.
Download RGB and Download depth PNG therefore share the same run ID, sequence
and settings revision. Fetching individual image routes separately can select
different pairs; use `/capture` when correspondence matters. Depth images are
sensor data, with calibration readiness still false.

POSTs retain the same-origin JSON guard. Send `expected_run_id` and
`expected_revision` from status plus at least one changed group, for example:

```json
{
  "expected_run_id": "<run ID from status>",
  "expected_revision": 0,
  "exposure": {"mode": "manual", "time_us": 8000, "iso": 200},
  "white_balance": {"mode": "manual", "temperature_k": 4500}
}
```

These example values are not a calibration recommendation. Each supplied group
replaces that entire group; Auto contains only `{"mode":"auto"}`. Manual
exposure requires integer `time_us` and `iso` together. AF devices accept
`focus: {"mode":"once"}` or `{"mode":"manual","lens_position":130}`;
fixed-focus devices refuse focus changes. Manual white balance requires integer
`temperature_k`. `anti_banding` accepts `off`, `50hz`, `60hz` or `auto` and
only affects auto-exposure; retaining it in Manual does not prevent flicker.

Capabilities publish supported modes and bounds. The adapter's application
policy accepts exposure 100–60000 µs (below the 5-fps frame period), ISO
100–1600, lens position 0–255 and white balance 1000–12000 K.
`range_source: "application_policy"` and `hardware_validated: false` distinguish
these software bounds from measured sensor limits. Values are never silently
clamped. Unknown/null fields, partial Manual groups and manual values included
in Auto are refused.

Settings admission shares a short guard with motion. Changes during movement
or an earlier pending change return 409; motion initiation waits until the
camera change has produced a post-dispatch fresh frame. SDK I/O holds neither
Session nor driver locks. Stop remains available throughout. There is no
calibration batch or autonomous motion implementation at this stage.

Acceptance returns `{"ok":true,"result":...}` with requested revision/groups
and `application_status: "queued"`. Status later reports `sent` or `failed`.
`sent` means SDK dispatch, not verified sensor application. Frame values
(`time_us`, `iso`, `temperature_k`, `lens_position`, dimensions and `sensor_fps`)
remain separate from requests; unknown lens position is null. `delivered_fps`
measures delivery to the producer. `settings_revision` identifies the requested
configuration; `settings_verified` and `measurement_ready` remain false.

Frames include run ID, sequence, UTC `captured_at` estimated from the SDK's
host-synchronized acquisition timestamp, `received_at` and acquisition age in
milliseconds. Images older than one second are refused. A settings change
clears the cache; pre-dispatch frames cannot acquire the new revision. Optional
snapshot query `after_run_id=<run>&after_sequence=<sequence>` awaits a newer
frame; both fields are required together. `wait_ms` defaults to 1000 and accepts
0–2000. Waiting is asynchronous and bounded, with no per-client USB or encoding.

Camera errors use 400 for invalid settings/query values, 409 for changed
run/revision or busy state, 503 for disabled/stale/missing output or snapshot
deadline, and 500 for unexpected camera handler failures. Camera errors do not
invoke Session Stop. The worker retries connection after failures and assigns
a new run ID and default settings after reconnect; old frame cursors/settings
versions then conflict. No usable frame for five seconds triggers reconnection.
The worker is a thread in the existing process; native SDK process failure
is not isolated by a separate camera process.

The Camera accordion provides capability-driven settings, requested/frame
readings, View/Pause, fresh Capture and JPEG Download. View starts on demand;
Pause closes that viewer's stream while acquisition continues. Camera feedback
and request state are independent of robot controls. Metric tracking,
settling/optical checks and calibration evidence are later work.

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
Poses, Session settings, Demo, Camera and Bounded tests. Swagger UI loads its scripts from a CDN
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
| 500 | Driver or server failure; Session actions stop first. Camera failures remain camera errors. |

Guard failures return 403. Invalid actions/arguments return 400. Driver/server
failures attempt direct stop and return 500. Unknown GET paths return 404.
No action response proves physical arrival: wait until state is idle and
inspect `error`, `phase`, `commands` and Demo progress. Commands remain
commanded counts, not measured position.

This API retains the existing commissioning actions. Future clients can extend
application actions without duplicating motion or bypassing readiness. See
[architecture](architecture.md). No agent adapter or agent-specific endpoint is
implemented here.
