# Contributing

Use `uv` and Python 3.12+ on macOS or Linux for development. Preview and tests
need only FastAPI and Uvicorn from `uv.lock`; hardware dependencies belong to the
isolated Pi runtime. Install [uv](https://docs.astral.sh/uv/getting-started/installation/) first.

```sh
uv sync --locked
uv run --locked python -m unittest discover -s tests -p 'test_*.py'
uv run --locked python scripts/manual_control.py
```

Tests use fake drivers and clocks. HTTP tests start real Uvicorn servers on loopback.
Connection tests mock SSH; deployment tests execute payloads against temporary
directories. Running the suite does not connect to a Pi or touch I2C.

Keep changes focused. Follow [architecture](docs/architecture.md): HTTP maps
requests to application actions, application code owns control decisions, and
drivers own hardware access. Do not add import-time hardware access or automatic
powered startup. Preserve count units, command/position distinctions, motion
ownership and failure/cancellation behavior.

For API changes, update [API](docs/api.md) and test validation, request guards,
state transitions and failures. For deployment changes, test secrets, strict
host verification, conflicts and preservation of Pi-side data. Update
`deploy.json` explicitly when adding operational files or assets.

Software tests do not establish physical success. Powered tests need a current
attending operator, supported arm, agreed targets/path, supply readiness and
reachable stop action. Record what the operator observed separately from fake
tests and register readback. See [operations](docs/operations.md).

Commit changes to both `pyproject.toml` and `uv.lock` when project dependencies
change. The Pi wheel lock remains platform-specific; use `uv pip` through the
runtime setup script to preserve its hashes. Do not sync the laptop project
lock into `.venv-runtime`, which contains the hardware dependencies.
