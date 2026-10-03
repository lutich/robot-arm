"""Validated Pi connection settings from .pi.json; no SSH or file access."""
from dataclasses import dataclass
from pathlib import PurePosixPath
import re


@dataclass(frozen=True)
class Connection:
    host: str = 'raspberrypi.local'
    user: str = 'pi'
    remote_dir: str = '/home/pi/robot-arm'
    ssh_port: int = 22
    app_port: int = 8765
    identity_file: str | None = None
    known_hosts: str = '.local/known_hosts'

    def __post_init__(self):
        if not isinstance(self.host, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*', self.host):
            raise ValueError('host must be a hostname or IPv4 address')
        if not isinstance(self.user, str) or not re.fullmatch(r'[a-z_][a-z0-9_-]*', self.user):
            raise ValueError('user must be a Linux login name')
        for name, port in (('ssh_port', self.ssh_port), ('app_port', self.app_port)):
            if type(port) is not int or not (1 if name == 'ssh_port' else 1024) <= port <= 65535:
                raise ValueError(f'Invalid {name}')
        if not isinstance(self.remote_dir, str) or any(c in self.remote_dir for c in '\n\r\x00'):
            raise ValueError('remote_dir must be an absolute Pi directory')
        path = PurePosixPath(self.remote_dir)
        if not path.is_absolute() or str(path) == '/' or '..' in path.parts:
            raise ValueError('remote_dir must be an absolute project directory without ..')
        if not isinstance(self.known_hosts, str) or not self.known_hosts:
            raise ValueError('known_hosts must name a project host-key file')
        if self.identity_file is not None and (not isinstance(self.identity_file, str) or not self.identity_file):
            raise ValueError('identity_file must be null or an SSH key path')
