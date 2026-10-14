# Arm API: one API for the simulated and the real arm

`threejs-sim/server.mjs` serves the robot's **low-level API**, the same one the controller on the Pi has (`lutich/robot-arm`, `docs/api.md`; summary in `reference-robot-arm-repo.md` section 2).
Behind it sits a backend that can be switched for everybody: a **shared simulated arm**, or the **real arm**.
Code written against this server runs unchanged on the simulation and on the real arm.
The simulator page is just one client of it.

```
 simulator page   your script   teammate's tool
        \             |              /
         ▼            ▼             ▼
   ┌─────────────────────────────────────────────┐
   │ server.mjs   /api/... (controller API)      │
   │              /api/backend  sim | real       │
   └──────┬───────────────────────────┬──────────┘
          ▼                           ▼
   shared simulated arm        real controller (Pi)
   (cubes, physics, collision) every motion call simulated first, blocked on collision
```

## Start

```powershell
cd threejs-sim
node server.mjs                                                         # simulation only, http://127.0.0.1:8770/
node server.mjs --controller http://10.102.4.103:8765                   # + the real arm, read only (stop allowed)
node server.mjs --controller http://10.102.4.103:8765 --allow-motion    # + motion to the real arm, after the sim check
node server.mjs --host 0.0.0.0 ...                                      # reachable from other machines
```

It always starts on the simulation. Switching to the real arm is one call, or the "Real arm" button in the page header.

## Calls

| Call | Body | What it does |
|------|------|--------------|
| `GET /api/state` | | Commanded counts, busy, outputs, poses, limits, demo; `backend` says which arm answered; the simulation also reports `position` |
| `POST /api/move` | `{"channel": 0, "target": 400}` | One joint, others hold (20 counts/s) |
| `POST /api/jog` | `{"channel": 0, "direction": 1}` | One step |
| `POST /api/go_pose` | `{"name": "HOME"}` | All joints to HOME or PARK |
| `POST /api/stop` | `{}` | Emergency stop, outputs off. Never blocked |
| `POST /api/power_on` | `{}` | Outputs on: PARK, then HOME |
| `POST /api/settings`, `/api/limits`, `/api/demo/...` | as on the controller | Same as on the controller |
| `GET /api/camera/…`, `GET /api/camera2/…` | | The two cameras (OAK, webcam): `status`, `snapshot.jpg`, `stream.mjpg`, `capture`. Forwarded read only from the controller with `--camera <url>` (or `--controller`); without it the OAK gives a saved frame and camera2 has none |
| `GET /api/backend` | | `{"target": "sim" \| "real", "real": "<url>", "allowMotion": …}` |
| `POST /api/backend` | `{"target": "real"}` | Switch for everybody. No login |
| `GET /sim/world`, `POST /sim/scene` | `{"cubes": [["orange", 0, 14, 0, 0], …]}` | The simulation's cubes ([colour, x, y, level, yaw], board frame cm). Not part of the robot API |

Rules, the same as the real controller's:
- POSTs need `Content-Type: application/json` and `Origin: http://<this server>`.
- Targets outside the limits are rejected (HTTP 400), not clamped.
- Only one motion at a time.

## Safety in real mode

- Every call that can move the arm is first played on a copy of the simulation, synced to the real arm's state. If the arm would hit itself or the board, the call is **not sent** and the answer is **HTTP 409** with the reason.
  - Exception: at PARK the gripper may touch the base, as the real one does.
- `stop` always goes through.
- Without `--allow-motion`, real mode is read only (state and stop).
- In simulation mode, a collision stops the simulated arm like an emergency stop (`outputs_off`); `power_on` re-arms it.

## Example (Python, standard library only)

```python
import json, urllib.request
BASE = "http://127.0.0.1:8770"

def call(path, body=None):
    req = urllib.request.Request(f"{BASE}/api/{path}", method="POST" if body is not None else "GET",
        data=None if body is None else json.dumps(body).encode(),
        headers={"Content-Type": "application/json", "Origin": BASE})
    return json.load(urllib.request.urlopen(req))

print(call("backend"))                          # {"target": "sim", ...}
call("move", {"channel": 0, "target": 400})     # runs on whichever arm is active
print(call("state")["commands"])
```

## Limits

- Calibration edits in the page (HOME angles, directions) are local to that browser; the server's simulation uses `js/config.js`.
- The real arm has no position feedback, so the simulation knows its commanded pose only. The cubes in the simulation are wherever they were last put (scenario reset, camera detection, `/sim/scene`).
