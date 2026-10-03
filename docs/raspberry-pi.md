# Prepare a Raspberry Pi

The bundled runtime targets the recorded Raspberry Pi 4 baseline: Raspberry Pi
OS Lite arm64 / Debian 13 Trixie with Python 3.13. Other boards, architectures
and Python versions have not been validated by this starter. The Python lock
does not reproduce OS packages or firmware.

## OS and access

For a new card, use Raspberry Pi Imager to configure the OS, hostname, login
user, Wi-Fi if needed, and SSH. Select your existing public key or enable
password authentication. See the official
[SSH guide](https://www.raspberrypi.com/documentation/computers/remote-access.html#ssh).
For an existing Pi, retain its working installation and credentials.

The example configuration uses `pi@raspberrypi.local`; set `.pi.json` to the
actual login and hostname or IPv4 address. Also update `remote_dir` if your
login's home is not `/home/pi`. Both computers need network access to each other.
For a direct Ethernet connection without working mDNS, use the Pi's actual
address; no owner's private network settings are included.

On the Pi console, enable I2C using `sudo raspi-config`, Interface Options,
I2C. Follow any restart instruction. Add your login to the `i2c` group if it is
missing, then log out and reconnect:

```sh
sudo usermod -aG i2c "$USER"
```

See the official
[configuration guide](https://www.raspberrypi.com/documentation/computers/configuration.html#i2c).
Setup checks `/dev/i2c-1` permissions; it does not scan the bus or initialize a
controller. Keep servo power off and the arm supported during setup.

## Install the runtime

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) on both
your laptop and Pi. The tooling was tested with uv 0.11.26. For an explicit
installation of that version on the Pi, run this in a Pi terminal:

```sh
curl -LsSf https://astral.sh/uv/0.11.26/install.sh | sh
```

Setup searches the Pi SSH PATH and `$HOME/.local/bin` for uv.
They do not install or upgrade uv automatically.

Configure SSH and deploy the reviewed files as described in
[deployment](deployment.md), then run this from your laptop:

```sh
uv run --locked python scripts/setup_pi.py --install-system
```

The optional flag runs `sudo -n apt-get install -y liblgpio1 i2c-tools`. It
requires available package indexes and noninteractive sudo permission. If
sudo or package indexes need attention, use a Pi terminal to install those
packages, then run `uv run --locked python scripts/setup_pi.py` without the flag.
There is no broad OS upgrade, firmware flash, reboot or service installation.

The setup script checks Linux/aarch64/Python 3.13, `liblgpio.so.1` and I2C access.
It then verifies bundled wheel hashes, creates the environment with `uv venv`
using system Python 3.13, and installs the pinned dependencies with `uv pip`
offline into `<remote_dir>/.venv-runtime`. An existing environment is checked
without installing or upgrading packages. Version or dependency mismatches
fail; repair separately or choose a new remote project directory.

For an on-Pi checkout, the same steps are available directly:

```sh
sh scripts/setup_pi.sh --install-system
```

Python runtime verification reads installed distribution metadata and runs
`uv pip check --python .venv-runtime/bin/python`. Neither pip nor a Python
download is needed. `uv.lock` describes the laptop project; the hash-locked
Pi wheels remain the authoritative hardware dependency set. Do not point
`uv sync` at `.venv-runtime`, because the laptop lock has no hardware packages.
Runtime setup does not instantiate the hardware driver or install the app
service; `scripts/service.py install` does that separately (see
[deployment](deployment.md)).
