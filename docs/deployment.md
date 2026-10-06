# Connect, deploy and launch

Run the laptop entry scripts from a checkout using `uv`, Python 3.12+ and OpenSSH
(`ssh`, `ssh-keyscan`, `ssh-keygen`). macOS and Linux are the supported tooling
targets. The shared CLI is `uv run --locked python scripts/pi.py <action>`; the entry scripts
provide the same actions separately. Run `uv sync --locked` first;
`uv run --locked` uses the project environment without updating the lockfile.

## 1. Configure access

```sh
cp pi.example.json .pi.json
```

Edit `.pi.json`: host, login user, absolute remote project directory, SSH port
and app port. Relative `known_hosts` and `identity_file` paths resolve against
the checkout root; `~` expands on the laptop. `remote_dir` is a Pi path.

For keys, install your public key through Pi Imager or the official
[SSH key instructions](https://www.raspberrypi.com/documentation/computers/remote-access.html#configure-ssh-without-a-password).
Use ssh-agent or set `identity_file` to an existing private key. Load a
passphrase-protected key into ssh-agent first. The scripts do not create keys
or replace the Pi's `authorized_keys`.

For password authentication, copy `.env.example` to `.env` and set `PI_PASS`
there, or provide `PI_PASS` in your process environment. Environment values take
precedence. The `.env` file is parsed as data; shell substitutions are not run.
Passwords go to SSH through a private askpass process, never command arguments.
Keep `.env`, `.pi.json` and `.local/` private; all are ignored by Git.

## 2. Verify the host key

On the Pi console or through an already trusted connection:

```sh
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

Use the reported `SHA256:...` value on your laptop:

```sh
uv run --locked python scripts/setup_connection.py --fingerprint 'SHA256:<verified fingerprint>'
```

The script scans an Ed25519 key, compares its fingerprint and only then pins it
in the project host-key file. Mismatches preserve existing keys. SSH always uses
strict checking and does not modify global known_hosts. Later connection tests
can omit `--fingerprint`. A replaced OS or changed key requires independently
verified connection configuration; the script does not overwrite an old key.

## 3. Deploy

Press Power off in the app first; deploy refuses while the arm is powered. Then:

```sh
uv run --locked python scripts/deploy.py
```

`deploy.json` is the explicit file list: operational Python files, frontend,
public reference records, guides, uv project files, Pi lock and wheels. Credentials, connection
settings, runtime environments and generated records are excluded. The script
checks every selected destination for conflicts before writing, replaces
individual files atomically and verifies SHA256 readback. It does not delete
files or mirror directories.

Before writing anything, deploy reads the app's state on the Pi and refuses with
`Arm is powered; press Power off before deploying` while outputs are on, because
a restart would drop the arm limp. If the app service is running and files
changed, deploy restarts it and waits for it to answer. A stopped service stays
stopped and picks up the files at its next start.

Each deploy records the hashes it wrote in `.deployed.json` in the Pi project
directory. A later deploy replaces a Pi file when it still matches that record,
so ordinary updates need no extra step; identical files are retained. A Pi file
changed since the last deploy, or any differing file when no record exists yet
(a Pi last deployed before this record existed), causes a refusal. To overwrite
such files deliberately, first save and review the read-only remote hash snapshot:

```sh
uv run --locked python scripts/check.py --snapshot .local/pi-before-update.json
```

Review the Pi-side changes and retain any work you need, then explicitly permit
those known previous bytes:

```sh
uv run --locked python scripts/deploy.py --baseline .local/pi-before-update.json
```

Only paths whose current remote hash matches the new version, the record or the
supplied baseline can change.
The snapshot uses a new filename and refuses an existing file. Reference JSON
evidence is immutable even with a baseline. Generated `artifacts/` data is never
part of deployment. Concurrent Pi edits can still race deployment; keep one
operator responsible for deploying while the arm is powered off.

For a focused update:

```sh
uv run --locked python scripts/deploy.py --files web/manual_control.css --baseline .local/pi-before-update.json
```

## 4. Setup and the app service

```sh
uv run --locked python scripts/setup_pi.py --install-system
uv run --locked python scripts/service.py install
```

The setup installs/verifies the runtime; see [Pi preparation](raspberry-pi.md).
`service.py install` enables lingering for the login user (one `sudo` using
`PI_PASS`, sent on SSH stdin) and installs the user unit
`~/.config/systemd/user/robot-arm.service`. The unit runs
`.venv-runtime/bin/python scripts/manual_control.py --hardware` with all six
channels, listening on every interface at `app_port`, and starts at boot.
Re-running install is safe. Review [operations](operations.md) before using it
with servo power.

Camera acquisition is disabled unless installation explicitly includes
`--camera oak`. After installing the [camera add-on](../runtime/README.md#optional-oak-camera-add-on),
with servo power off and the arm supported, install the camera-enabled unit:

```sh
uv run --locked python scripts/service.py stop
uv run --locked python scripts/service.py install --camera oak
```

Stopping first ensures the changed startup command takes effect. The camera
choice persists across service restarts; repeat `--camera oak` when reinstalling.
Installing without it restores the default camera-disabled command.

Camera-enabled units use `KillMode=mixed`: systemd first signals the parent app,
which stops the arm and closes its separate camera process; systemd retains
whole-group forced cleanup after the 15-second stop budget. After upgrading
from the thread-based camera worker, repeat the stop/install commands above
to update the existing unit as well as the deployed Python files. Verify fresh
RGB/depth, a new camera run ID and clean service shutdown with servo power off.

Open http://raspberrypi.local:8765/ (or the Pi's IP address) from any device on
the Pi's Wi-Fi or Ethernet network; API state is at `/api/state`. Substitute your
configured `app_port`. The app starts unprepared/unarmed: it does not initialize
I2C or send PWM until Power on in the UI. There is no authentication; use it
only on a network you control.

```sh
uv run --locked python scripts/service.py status    # state, recent log lines and URL
uv run --locked python scripts/service.py stop      # verified PWM off; the arm goes limp
uv run --locked python scripts/service.py start
uv run --locked python scripts/service.py restart
```

Stop the service before running `joint_check.py` or `gripper_check.py` on the
Pi: after a Power on, the app holds the controller until it exits. The service
does not restart itself after a crash or an unconfirmed PWM-off; `status` shows
why it stopped.

### Updating a Pi installed before FastAPI

The runtime installer never upgrades an existing `.venv-runtime` in place, so a
Pi set up before the FastAPI change needs its runtime rebuilt once:

1. Press Power off, then run `uv run --locked python scripts/service.py stop`.
2. Run `uv run --locked python scripts/deploy.py`; a stopped service stays stopped.
3. On the Pi, keep the old runtime aside:
   `mv ~/robot-arm/.venv-runtime ~/robot-arm/.venv-runtime-pre-fastapi`
   (substitute your `remote_dir`).
4. Run `uv run --locked python scripts/setup_pi.py`; it installs the new runtime
   offline from the bundled wheels.
5. Run `uv run --locked python scripts/service.py start`, then
   `uv run --locked python scripts/check.py --app`.

To roll back, stop the service, swap the two runtime folders back and deploy the
previous commit.

## 5. Check

In another laptop terminal:

```sh
uv run --locked python scripts/check.py --app
```

The check reports remote file hashes, uv availability, runtime versions/
dependency health, I2C
device access, served frontend hashes and API state. It uses only HTTP GETs;
it does not prepare hardware or refresh readiness. Without `--app`, it
checks the installed files/runtime without requiring a running app. Exit 1
indicates a mismatch or missing prerequisite. Snapshot mode saves hashes for
review even when the deployment does not yet match.

Use `--config path/to/connection.json` on any entry script for another Pi.

## Wi-Fi network

Set `WIFI_SSID` and `WIFI_PASS` (WPA-Personal) in `.env`, then:

```sh
uv run --locked python scripts/setup_wifi.py
```

This saves or updates one NetworkManager profile for that SSID with priority 10
as a root-only keyfile, using `PI_PASS` for `sudo` on the Pi. The Wi-Fi password
travels on SSH stdin, never in command arguments. Existing profiles are kept as
fallbacks. The Pi uses the new network at the next boot or when the current one
disappears. `--connect` switches 3 s after the command returns; an SSH session
over Wi-Fi then drops, so use the Ethernet cable or join the laptop to the new
network. A wrong password falls back to another saved network when one is in range.
`raspberrypi.local` resolves only on the network the laptop shares with the Pi.

## Shutdown and connection failures

For hardware, complete Power off with support and a clear return path, then
turn servo supply off manually. The app keeps running on the Pi: closing the
browser or the laptop, or losing the network, does not stop it or PWM.
`service.py stop` signals the app, which disables PWM and checks readback before
it exits; `service.py status` then shows it inactive. Establish support and use
the physical servo power switch if cleanup cannot be confirmed. See
[operations](operations.md).

If SSH fails, check the configured address, credentials and independently
verified host key. If setup is interrupted, inspect the Pi runtime before
retrying; an SSH timeout is not proof that the remote command stopped.
