"""Configurable SSH with a project host-key file and optional password askpass."""
import os
from pathlib import Path
import json
import re
import subprocess

from roboter_arm.provisioning.domain.connection import Connection
from roboter_arm.provisioning.infrastructure.env import read_password
from roboter_arm.shared.paths import ROOT

# Executable launcher that answers SSH password prompts; see its ROBOT_ASKPASS branch.
ASKPASS = ROOT / 'scripts/pi_access.py'


def local_path(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else ROOT / path


def arguments(connection, *, password=False):
    args = ['ssh', '-4', '-T', '-p', str(connection.ssh_port),
            '-o', f'UserKnownHostsFile={local_path(connection.known_hosts)}',
            '-o', 'GlobalKnownHostsFile=/dev/null', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=10', '-o', 'NumberOfPasswordPrompts=1',
            '-o', 'BatchMode=no' if password else 'BatchMode=yes']
    if connection.identity_file:
        args += ['-i', str(local_path(connection.identity_file)), '-o', 'IdentitiesOnly=yes']
    return args + [f'{connection.user}@{connection.host}']


def ssh(connection, command, payload=b'', *, timeout=60):
    password = read_password()
    env = os.environ.copy()
    if password:
        env.update(SSH_ASKPASS=str(ASKPASS), SSH_ASKPASS_REQUIRE='force',
                   DISPLAY='robot-ssh', ROBOT_ASKPASS='1', ROBOT_SSH_PASSWORD=password)
    args = arguments(connection, password=bool(password)) + [command]
    return subprocess.run(args, input=payload, capture_output=True, env=env,
                          start_new_session=True, timeout=timeout)


def load_connection(path=None):
    path = Path(path) if path else ROOT / '.pi.json'
    if not path.is_file():
        raise ValueError('Copy pi.example.json to .pi.json and configure your Pi first')
    values = json.loads(path.read_text())
    if not isinstance(values, dict) or set(values) - Connection.__dataclass_fields__.keys():
        raise ValueError('Connection config contains unknown fields')
    return Connection(**values)


def setup_connection(connection, fingerprint=None):
    """Pin a scanned Ed25519 key only after matching an independently known hash."""
    destination = local_path(connection.known_hosts)
    if fingerprint is not None:
        if not re.fullmatch(r'SHA256:[A-Za-z0-9+/]{43}', fingerprint):
            raise ValueError('Expected an independently verified SHA256 host-key fingerprint')
        scan = subprocess.run(['ssh-keyscan', '-4', '-T', '10', '-p', str(connection.ssh_port),
                               '-t', 'ed25519', connection.host], capture_output=True, timeout=15)
        keys = list(dict.fromkeys(line for line in scan.stdout.splitlines() if line and not line.startswith(b'#')))
        if scan.returncode or len(keys) != 1 or len(keys[0].split()) != 3 or keys[0].split()[1] != b'ssh-ed25519':
            raise ValueError('Could not obtain a single Ed25519 host key')
        key = keys[0]
        checked = subprocess.run(['ssh-keygen', '-lf', '-'], input=key + b'\n',
                                 capture_output=True, timeout=10)
        fields = checked.stdout.decode().split()
        if checked.returncode or len(fields) < 2 or fields[1] != fingerprint:
            raise ValueError('Pi host-key fingerprint does not match; no key was saved')
        existing = destination.read_bytes().splitlines() if destination.exists() else []
        host = key.split()[0]
        for line in existing:
            if line and not line.startswith(b'#') and host in line.split()[0].split(b',') and line != key:
                raise ValueError('A different Pi host key is already pinned; existing file preserved')
        if key not in existing:
            destination.parent.mkdir(parents=True, exist_ok=True)
            with destination.open('ab') as stream:
                if destination.stat().st_size and not destination.read_bytes().endswith(b'\n'):
                    stream.write(b'\n')
                stream.write(key + b'\n')
    elif not destination.is_file():
        raise ValueError('First connection requires --fingerprint from the Pi console or another trusted source')
    return ssh(connection, 'python3 --version')

