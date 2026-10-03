# Laptop preview

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and use
Python 3.12+ with a browser. Preview needs only FastAPI and Uvicorn, which
`uv sync` installs; the native Pi wheels are not needed on your laptop. `uv` manages the local
`.venv` using `pyproject.toml` and `uv.lock`.

From the checkout root:

```sh
uv sync --locked
uv run --locked python scripts/manual_control.py
```

Open http://127.0.0.1:8765/. The footer must identify **Preview · no hardware**.
The simulated PARK/readiness checkbox allows Power on, which enables simulated
PARK commands and moves to HOME. Select a joint, jog or submit an integer target,
record Demo positions, then try Next/Run/Pause. Power off returns the simulated
commands to PARK and disables; Emergency stop disables directly.

Targets and step size are PCA9685 counts at nominal 60 Hz. They are not degrees.
Preview tests the control workflow; it cannot establish reachability or
mechanical clearance.

GET http://127.0.0.1:8765/api/state returns state without changing readiness or
initializing hardware. Interactive API docs (Swagger UI) are at
http://127.0.0.1:8765/docs. For action request bodies and the same-origin rule, see
[API](api.md).

Stop with Ctrl+C. Synthetic poses and demos are written to ignored
`artifacts/poses/synthetic/` and `artifacts/demos/synthetic/`. They cannot be
loaded as hardware Demo records.

If port 8765 is occupied, stop the other app or choose a different port:

```sh
uv run --locked python scripts/manual_control.py --port 8766
```

Laptop preview listens on this machine only. The Pi app is a separate service
on the Pi; see [deployment](deployment.md).
