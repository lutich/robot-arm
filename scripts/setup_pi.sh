#!/bin/sh
# OS prerequisites and isolated runtime only; no controller access or autostart.
set -eu
base=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
export PATH="$HOME/.local/bin:$PATH"
command -v uv >/dev/null 2>&1 || { echo 'Install uv on the Pi first; see docs/raspberry-pi.md.' >&2; exit 1; }
if [ "${1:-}" = "--install-system" ]; then
    sudo -n apt-get install -y liblgpio1 i2c-tools
elif [ "$#" -ne 0 ]; then
    echo 'Usage: setup_pi.sh [--install-system]' >&2
    exit 2
fi
python3 - <<'PY'
import ctypes, os, platform, sys
assert platform.system() == 'Linux' and platform.machine() == 'aarch64' and sys.version_info[:2] == (3, 13), 'Requires Raspberry Pi OS arm64 / Python 3.13'
ctypes.CDLL('liblgpio.so.1')
assert os.access('/dev/i2c-1', os.R_OK | os.W_OK), 'Enable I2C and grant your login user i2c access; reconnect SSH afterward'
PY
sh "$base/scripts/install_runtime.sh"
