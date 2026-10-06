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

## Optional OAK camera add-on

Camera access is opt-in. After installing the bundled runtime, stop the app and
run this on the Pi with internet access:

```sh
sh scripts/install_camera_runtime.sh
```

The add-on pins [DepthAI 3.6.1](https://pypi.org/project/depthai/3.6.1/#files)
and its only Python dependency,
[NumPy 2.4.4](https://pypi.org/project/numpy/2.4.4/#files), in
`requirements/camera-py313.lock`. Only the published binary wheels for Linux
aarch64, standard CPython 3.13 and glibc 2.28+ are accepted by SHA256. These two
wheels are downloaded from PyPI; they are not part of the offline arm bundle.
The original runtime installer and bundled wheel lock keep their offline
behavior.

The camera installer requires an existing verified arm runtime, checks its
package pins before and after installation, and refuses unexpected existing
DepthAI or NumPy versions. An existing 3.10.0 camera add-on can be replaced with
the tested 3.6.1 pin explicitly, after stopping the app:

```sh
sh scripts/install_camera_runtime.sh --replace-camera-sdk
```

Other SDK or NumPy replacements are refused. It installs only these two packages with dependency
resolution disabled, then checks the resulting dependencies. It never imports
the SDK or connects to a camera or controller. Preview and software tests do
not require this add-on.

The reference OAK-D Lite failed to start stereo with DepthAI 3.10.0, reporting
`PlgSrcMipi.cpp` and `RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE`. An isolated 3.6.1
test on 2026-10-06 produced 149 aligned RGB/depth pairs in 29.29 seconds
(5.05 fps), then shut down cleanly in 2.01 seconds. This is the reason for the
SDK pin. The pipeline disables automatic recalibration and uses the camera's
existing calibration; capture does not request calibration writes.
The deployed API subsequently passed a 120-second simultaneous RGB/depth
stream check at 5.007 fps. Raw depth decoded as uint16 millimetres and matched
the capture metadata. Long-run reliability and full setting readbacks remain
unverified.
See the stage 0 evidence in `PLAN-camera-calibration.md` in the laptop checkout.

Linux also needs camera USB permissions. On this reference Pi an explicitly
approved `/etc/udev/rules.d/80-robot-arm-oak.rules` grants login `pi` access to
USB vendor `03e7` with mode `0660`. The package installer does not create this
system rule; other installations need their own reviewed USB access setup.
