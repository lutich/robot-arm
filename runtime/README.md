# Bundled Pi runtime

21 of the 34 wheels are the verified dependency closure from the existing arm's
Raspberry Pi OS Trixie / Linux aarch64 / CPython 3.13 runtime. The other 13 are
the web stack (FastAPI, Starlette, Pydantic, Uvicorn and their dependencies),
downloaded from PyPI as binary wheels for this platform and checked against
PyPI's published SHA256 hashes. Only `pydantic-core` is compiled. They were
fetched with:

```sh
pip download --no-deps --only-binary=:all: --platform manylinux_2_28_aarch64 \
    --platform manylinux_2_17_aarch64 --platform manylinux2014_aarch64 \
    --python-version 3.13 --implementation cp --abi cp313 -d runtime/wheels <name==version ...>
```

The laptop `uv.lock` pins the same versions; `tests/test_runtime.py` checks this. The small bundle
is included so a fresh checkout can install the tested Python environment
without fetching or rebuilding native packages.

`requirements/pi-py313.lock` pins versions and wheel SHA256 hashes.
`runtime/wheels-manifest.json` identifies the corresponding filenames and hashes.
Three native wheels (`lgpio`, `RPi.GPIO`, `rpi_ws281x`) were built on the reference
Pi. Byte-identical source rebuilds are not claimed. The bundle is not an OS
image and is unsuitable for laptop installation or other Python/CPU targets.

Third-party package metadata and license files remain inside the unchanged
wheel archives. See [NOTICE](NOTICE.md) for their recorded metadata. The root
MIT license applies to the starter's own code; dependency licenses remain their
own. The reference closure is otherwise preserved unchanged.

Install uv on the Pi, then use `sh scripts/install_runtime.sh` on a matching
Pi, or `uv run --locked python scripts/setup_pi.py` on the laptop after
deployment. The installer uses `uv venv` and `uv pip install --require-hashes`
with the existing Pi wheel lock; `uv.lock` governs the laptop environment.
Existing environments are verified without package updates. Installation
checks metadata/dependencies and does not instantiate a controller.
