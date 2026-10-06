#!/bin/sh
# Opt-in online camera add-on; preserve the existing bundled arm runtime.
set -eu
base=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
usage() { echo 'Usage: install_camera_runtime.sh [target] [--replace-camera-sdk]' >&2; exit 2; }
target="$base/.venv-runtime"
target_given=false
replace_camera_sdk=false
for argument do
    case "$argument" in
        --replace-camera-sdk)
            [ "$replace_camera_sdk" = false ] || usage
            replace_camera_sdk=true ;;
        ''|-*) usage ;;
        *)
            [ "$target_given" = false ] || usage
            target="$argument"
            target_given=true ;;
    esac
done
export PATH="$PATH:$HOME/.local/bin"
[ -x "$target/bin/python" ] || { echo 'Install the bundled runtime first with scripts/install_runtime.sh.' >&2; exit 1; }
"$target/bin/python" - "$base/requirements/camera-py313.lock" "$replace_camera_sdk" <<'PY'
import importlib.metadata as metadata, os, pathlib, platform, sys, sysconfig
assert platform.system() == 'Linux' and platform.machine() == 'aarch64', 'Requires aarch64 Linux'
assert platform.python_implementation() == 'CPython' and sys.version_info[:2] == (3, 13), 'Requires CPython 3.13'
assert not sysconfig.get_config_var('Py_GIL_DISABLED'), 'Requires standard CPython, not a free-threaded build'
libc = os.confstr('CS_GNU_LIBC_VERSION').split()
assert libc[0] == 'glibc' and tuple(map(int, libc[1].split('.')[:2])) >= (2, 28), 'Requires glibc >= 2.28'
for line in pathlib.Path(sys.argv[1]).read_text().splitlines():
    if line and not line.startswith('#'):
        name, version = line.split()[0].split('==')
        try:
            existing = metadata.version(name)
        except metadata.PackageNotFoundError:
            continue
        replacement = (sys.argv[2] == 'true' and name == 'depthai'
                       and existing == '3.10.0' and version == '3.6.1')
        assert existing == version or replacement, f'Existing {name}=={existing} differs; camera installer will not replace it'
PY
sh "$base/scripts/install_runtime.sh" "$target"
uv pip install --python "$target/bin/python" --no-python-downloads --only-binary=:all: --require-hashes --no-deps -r "$base/requirements/camera-py313.lock"
uv pip check --python "$target/bin/python" --no-python-downloads
sh "$base/scripts/install_runtime.sh" "$target"
"$target/bin/python" - "$base/requirements/camera-py313.lock" <<'PY'
import importlib.metadata as metadata, pathlib, sys
for line in pathlib.Path(sys.argv[1]).read_text().splitlines():
    if line and not line.startswith('#'):
        name, version = line.split()[0].split('==')
        assert metadata.version(name) == version, 'Camera runtime differs: ' + name
print('Pinned camera add-on verified; no camera or controller was initialized.')
PY
