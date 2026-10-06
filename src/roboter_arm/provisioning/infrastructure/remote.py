"""Programs and command lines run on the Pi over SSH, and their local preconditions."""
import json
import re
import shlex

from roboter_arm.provisioning.domain.wifi import check_wifi
from roboter_arm.provisioning.infrastructure.env import read_env, read_password


SERVICE = 'robot-arm.service'
# Non-interactive SSH may not set the user manager's runtime directory.
SYSTEMCTL = 'XDG_RUNTIME_DIR=/run/user/$(id -u) systemctl --user '


def service_unit(connection, *, camera=None):
    """User unit serving the hardware app on every interface; no PWM until Power on in the page."""
    if camera not in (None, 'oak'):
        raise ValueError('Service camera must be oak or omitted')
    root = connection.remote_dir
    if not re.fullmatch(r'[A-Za-z0-9._/-]+', root):
        raise ValueError('remote_dir must be a plain path (letters, digits, . _ - /) for the app service')
    command = ' '.join([f'{root}/.venv-runtime/bin/python', 'scripts/manual_control.py',
                        '--hardware', '--channels', '0', '1', '2', '3', '4', '5',
                        '--port', str(connection.app_port), '--host', '0.0.0.0'])
    if camera is not None:
        command += ' --camera ' + camera
    # Let the app close its camera worker before systemd escalates to the whole group.
    kill_mode = 'KillMode=mixed\n' if camera is not None else ''
    # Restart=no keeps a crash or an unconfirmed PWM-off visible in the service status.
    return ('[Unit]\nDescription=Robot arm control app\n\n'
            f'[Service]\nWorkingDirectory={root}\nExecStart={command}\nRestart=no\n{kill_mode}TimeoutStopSec=15\n\n'
            '[Install]\nWantedBy=default.target\n')


# Lingering starts the user's units at boot. The password is the first stdin line, so it is never
# a process argument; the unit text follows it.
UNIT_WRITER = r'''import os, pathlib, sys, tempfile
directory = pathlib.Path.home() / '.config/systemd/user'
directory.mkdir(parents=True, exist_ok=True)
with tempfile.NamedTemporaryFile('w', dir=directory, delete=False) as stream:
    stream.write(sys.stdin.read())
os.replace(stream.name, directory / 'robot-arm.service')
'''
INSTALL_COMMAND = (r'''IFS= read -r pw && printf '%s\n' "$pw" | sudo -S -k -p '' loginctl enable-linger "$(id -un)" && '''
                   'python3 -c ' + shlex.quote(UNIT_WRITER) + ' && ' + SYSTEMCTL + 'daemon-reload && '
                   + SYSTEMCTL + 'enable --now ' + SERVICE)


def install_payload(connection, *, camera=None):
    password = read_password()
    if not password:
        raise ValueError('service install needs PI_PASS in .env for sudo on the Pi')
    return (password + '\n' + service_unit(connection, camera=camera)).encode()


def service_command(action):
    if action not in ('start', 'stop', 'restart', 'status'):
        raise ValueError('Unknown service action')
    return SYSTEMCTL + action + (' --no-pager ' if action == 'status' else ' ') + SERVICE


def setup_command(connection, *, install_system=False):
    return f'cd {shlex.quote(connection.remote_dir)} && sh scripts/setup_pi.sh' + (' --install-system' if install_system else '')


# Runs as root. Settings arrive as the last stdin line, so the Wi-Fi password is never a process
# argument. NetworkManager loads root-only keyfiles from this directory; the UUID derives from the
# SSID, so re-running updates the same profile and other profiles stay as fallbacks.
WIFI = r'''import json, os, subprocess, sys, uuid
wifi = json.loads(sys.stdin.read().splitlines()[-1])
ssid, psk = (text.replace('\\', '\\\\').replace(' ', '\\s') for text in (wifi['ssid'], wifi['psk']))
profile = str(uuid.uuid5(uuid.NAMESPACE_URL, 'robot-arm-wifi:' + wifi['ssid']))
path = '/etc/NetworkManager/system-connections/robot-wifi-' + profile + '.nmconnection'
lines = ['[connection]', 'id=' + ssid, 'uuid=' + profile, 'type=wifi', 'autoconnect-priority=10',
         '[wifi]', 'mode=infrastructure', 'ssid=' + ssid, '[wifi-security]', 'key-mgmt=wpa-psk', 'psk=' + psk,
         '[ipv4]', 'method=auto', '[ipv6]', 'method=auto']
with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600), 'w') as stream:
    stream.write('\n'.join(lines) + '\n')
if subprocess.run(['nmcli', 'connection', 'load', path]).returncode:
    sys.exit('NetworkManager rejected the Wi-Fi profile')
print('Saved Wi-Fi profile ' + wifi['ssid'] + ' with priority 10; used at next boot or when the current network is gone.')
if wifi['connect']:
    subprocess.run(['systemd-run', '--quiet', '--on-active=3', 'nmcli', 'connection', 'up', 'uuid', profile], check=True)
    print('Switching to ' + wifi['ssid'] + ' in 3 s; an SSH session over Wi-Fi will drop.')
'''
# sudo is verified first, so a wrong PI_PASS never makes it read the settings line as a retry.
WIFI_COMMAND = (r'''IFS= read -r pw && printf '%s\n' "$pw" | sudo -S -k -p '' true && '''
                r'''{ printf '%s\n' "$pw"; cat; } | sudo -S -k -p '' python3 -c ''' + shlex.quote(WIFI))


def wifi_payload(*, connect=False):
    password, ssid, psk = read_password(), read_env('WIFI_SSID'), read_env('WIFI_PASS')
    if not password:
        raise ValueError('setup-wifi needs PI_PASS in .env for sudo on the Pi')
    check_wifi(ssid, psk)
    return (password + '\n' + json.dumps({'ssid': ssid, 'psk': psk, 'connect': connect}) + '\n').encode()
