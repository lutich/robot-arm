#!/bin/sh
# Recreate or verify the tested Raspberry Pi environment from bundled wheels.
set -eu
base=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
target=${1:-"$base/.venv-runtime"}
export PATH="$HOME/.local/bin:$PATH"
command -v uv >/dev/null 2>&1 || { echo 'Install uv on the Pi first; see docs/raspberry-pi.md.' >&2; exit 1; }
python3 -c 'import platform, sys; assert platform.system() == "Linux" and platform.machine() == "aarch64" and sys.version_info[:2] == (3, 13), "Requires aarch64 Linux and Python 3.13"'
python3 -c 'import ctypes; ctypes.CDLL("liblgpio.so.1")'
python3 - "$base" <<'PY'
import hashlib, json, pathlib, sys
root = pathlib.Path(sys.argv[1])
for wheel in json.loads((root / 'runtime/wheels-manifest.json').read_text()):
    assert hashlib.sha256((root / 'runtime/wheels' / wheel['file']).read_bytes()).hexdigest() == wheel['sha256'], 'Wheel hash mismatch: ' + wheel['file']
PY
if [ ! -e "$target" ]; then
    uv venv --python python3 --no-python-downloads "$target"
    uv pip install --python "$target/bin/python" --no-python-downloads --offline --no-index --find-links "$base/runtime/wheels" --require-hashes -r "$base/requirements/pi-py313.lock"
fi
uv pip check --python "$target/bin/python" --no-python-downloads
"$target/bin/python" - "$base/requirements/pi-py313.lock" <<'PY'
import importlib.metadata as metadata, pathlib, sys
assert sys.version_info[:2] == (3, 13), 'Existing environment must use Python 3.13'
for line in pathlib.Path(sys.argv[1]).read_text().splitlines():
    if line and not line.startswith('#'):
        name, version = line.split()[0].split('==')
        assert metadata.version(name) == version, 'Existing runtime differs: ' + name
print('Pinned runtime verified; no controller was initialized.')
PY
