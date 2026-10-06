"""Connection trust, secret handling and the app service; no live SSH."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.provisioning.domain.connection import Connection
from roboter_arm.provisioning.infrastructure import ssh
from roboter_arm.provisioning.infrastructure.remote import (
    INSTALL_COMMAND, WIFI, WIFI_COMMAND, install_payload, service_command, service_unit, setup_command, wifi_payload)
from roboter_arm.provisioning.presentation.cli import main


class ConnectionTests(unittest.TestCase):
    def test_configuration_and_unsafe_hosts_paths_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'connection.json'
            path.write_text(json.dumps(dict(host='192.168.1.5', user='operator', remote_dir='/home/operator/arm', app_port=8766)))
            connection = ssh.load_connection(path)
            self.assertEqual(connection.app_port, 8766)
            path.write_text('{"password": "not-supported"}')
            with self.assertRaises(ValueError):
                ssh.load_connection(path)
        for values in ({'host':'-oProxyCommand=bad'}, {'user':'pi;bad'}, {'remote_dir':'/'},
                       {'remote_dir':'/home/pi/../etc'}, {'app_port':True}, {'ssh_port':0}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Connection(**values)

    def test_password_file_is_data_and_environment_takes_precedence(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            path = Path(directory) / '.env'
            path.write_text('OTHER=ignored\nPI_PASS="$(never_execute) secret"\n')
            self.assertEqual(ssh.read_password(path), '$(never_execute) secret')
            os.environ['PI_PASS'] = 'from-environment'
            self.assertEqual(ssh.read_password(path), 'from-environment')
            os.environ['PI_PASS'] = ''
            self.assertEqual(ssh.read_password(path), '')

    def test_key_access_is_explicit_and_nothing_is_forwarded(self):
        connection = Connection(host='pi-one.local', user='operator', ssh_port=2222,
                                app_port=8766, identity_file='/tmp/robot-test-key')
        with patch.object(ssh, 'read_password', return_value=None), patch.object(subprocess, 'run') as run:
            ssh.ssh(connection, 'python3 --version')
        args, kwargs = run.call_args
        self.assertIn('BatchMode=yes', args[0])
        self.assertIn('StrictHostKeyChecking=yes', args[0])
        self.assertNotIn('-L', args[0])
        self.assertIn('/tmp/robot-test-key', args[0])
        self.assertIn('operator@pi-one.local', args[0])
        self.assertEqual(args[0][args[0].index('-p')+1], '2222')

    def test_password_not_in_arguments_and_payload_passed_on_stdin(self):
        with patch.object(ssh, 'read_password', return_value='test-secret'), patch.object(subprocess, 'run') as run:
            ssh.ssh(Connection(), 'python3 -', b'passive script')
        args, kwargs = run.call_args
        self.assertNotIn('test-secret', ' '.join(args[0]))
        self.assertEqual(kwargs['env']['ROBOT_SSH_PASSWORD'], 'test-secret')
        self.assertEqual(kwargs['input'], b'passive script')
        self.assertTrue(kwargs['start_new_session'])

    def test_setup_wifi_sends_secrets_on_stdin_only(self):
        settings = {'PI_PASS': 'login-secret', 'WIFI_SSID': 'Lab Net', 'WIFI_PASS': 'wifi secret 1'}
        with patch.dict(os.environ, settings), patch('roboter_arm.provisioning.presentation.cli.load_connection', return_value=Connection()), \
                patch.object(subprocess, 'run') as run, patch.object(sys, 'stdout'), patch.object(sys, 'stderr'):
            run.return_value = subprocess.CompletedProcess([], 0, b'', b'')
            with self.assertRaises(SystemExit) as exit:
                main(['setup-wifi', '--connect'])
        self.assertEqual(exit.exception.code, 0)
        args, kwargs = run.call_args
        self.assertNotIn('login-secret', ' '.join(args[0]))
        self.assertNotIn('wifi secret 1', ' '.join(args[0]))
        password, line = kwargs['input'].decode().splitlines()
        self.assertEqual(password, 'login-secret')
        self.assertEqual(json.loads(line), {'ssid': 'Lab Net', 'psk': 'wifi secret 1', 'connect': True})

    def test_setup_wifi_rejects_missing_or_unsupported_settings(self):
        valid = {'PI_PASS': 'login-secret', 'WIFI_SSID': 'Lab', 'WIFI_PASS': 'long enough'}
        for override in ({'PI_PASS': ''}, {'WIFI_SSID': ''}, {'WIFI_SSID': 'a;b'}, {'WIFI_SSID': 'x' * 33},
                         {'WIFI_PASS': 'short'}, {'WIFI_PASS': 'tab\tinside'}):
            with self.subTest(override=override), patch.dict(os.environ, {**valid, **override}), \
                    self.assertRaises(ValueError):
                wifi_payload()

    def test_wifi_settings_reach_root_only_after_sudo_accepts_password(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / 'capture'
            sudo = Path(directory) / 'sudo'
            # Like sudo -S -k -p '': read one password line, then run the command with the remaining stdin.
            sudo.write_text('#!/bin/sh\nIFS= read -r pw\n[ "$pw" = right ] || exit 1\n'
                            '[ "$5" = true ] && exit 0\ncat > "$CAPTURE"\n')
            sudo.chmod(0o755)
            env = dict(os.environ, PATH=f'{directory}:{os.environ["PATH"]}', CAPTURE=str(capture))
            for password, returncode, received in (('wrong', 1, False), ('right', 0, True)):
                result = subprocess.run(['sh', '-c', WIFI_COMMAND], input=f'{password}\n{{"psk": "x"}}\n',
                                        text=True, env=env, capture_output=True)
                self.assertEqual(result.returncode, returncode)
                self.assertEqual(capture.exists() and capture.read_text(), received and '{"psk": "x"}\n')

    def test_wifi_profile_is_private_keyfile_updated_in_place_and_activated(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'log'
            for tool in ('nmcli', 'systemd-run'):
                (Path(directory) / tool).write_text(f'#!/bin/sh\necho {tool} "$@" >> "{log}"\n')
                (Path(directory) / tool).chmod(0o755)
            script = WIFI.replace('/etc/NetworkManager/system-connections/', directory + '/')
            settings = json.dumps({'ssid': 'Lab Net', 'psk': 'a b\\c d', 'connect': True}) + '\n'
            for _ in range(2):
                result = subprocess.run([sys.executable, '-c', script], input=settings, text=True, capture_output=True,
                                        env=dict(os.environ, PATH=f'{directory}:{os.environ["PATH"]}'))
                self.assertEqual(result.returncode, 0, result.stderr)
            [profile] = Path(directory).glob('robot-wifi-*.nmconnection')
            self.assertEqual(profile.stat().st_mode & 0o777, 0o600)
            self.assertIn('\nssid=Lab\\sNet\n', profile.read_text())
            self.assertIn('\npsk=a\\sb\\\\c\\sd\n', profile.read_text())
            calls = log.read_text().splitlines()
            self.assertEqual(calls[0], f'nmcli connection load {profile}')
            self.assertTrue(calls[1].startswith('systemd-run --quiet --on-active=3 nmcli connection up uuid '))

    def test_setup_command_keeps_the_remote_directory_literal(self):
        root = "/home/pi/arm ' ; $(touch escaped)"
        command = setup_command(Connection(remote_dir=root))
        self.assertEqual(shlex.split(command)[:2], ['cd', root])
        self.assertNotIn('apt-get', command)

    def test_service_unit_runs_the_hardware_app_on_every_interface(self):
        unit = service_unit(Connection(remote_dir='/home/pi/robot-arm', app_port=8766)).splitlines()
        self.assertIn('WorkingDirectory=/home/pi/robot-arm', unit)
        self.assertIn('ExecStart=/home/pi/robot-arm/.venv-runtime/bin/python scripts/manual_control.py '
                      '--hardware --channels 0 1 2 3 4 5 --port 8766 --host 0.0.0.0', unit)
        self.assertIn('Restart=no', unit)
        self.assertIn('WantedBy=default.target', unit)
        for root in ("/home/pi/arm ' ; $(touch escaped)", '/home/pi/arm%n', '/home/pi/a$HOME'):
            with self.subTest(root=root), self.assertRaises(ValueError):
                service_unit(Connection(remote_dir=root))

    def test_service_commands_use_the_user_manager(self):
        for action in ('start', 'stop', 'restart', 'status'):
            with self.subTest(action=action):
                command = service_command(action)
                self.assertTrue(command.startswith('XDG_RUNTIME_DIR=/run/user/$(id -u) systemctl --user ' + action))
                self.assertTrue(command.endswith(' robot-arm.service'))
        self.assertIn('--no-pager', service_command('status'))
        with self.assertRaises(ValueError):
            service_command('enable; reboot')

    def test_service_camera_is_explicit_and_validated(self):
        default = service_unit(Connection())
        camera = service_unit(Connection(), camera='oak')
        self.assertNotIn('--camera', default)
        self.assertEqual(camera, default.replace('--host 0.0.0.0', '--host 0.0.0.0 --camera oak')
                         .replace('TimeoutStopSec=15', 'KillMode=mixed\nTimeoutStopSec=15'))
        for value in ('usb', 'oak; reboot', True):
            with self.subTest(camera=value), self.assertRaises(ValueError):
                service_unit(Connection(), camera=value)

    def test_camera_install_sends_opt_in_and_secret_on_stdin_only(self):
        with patch.dict(os.environ, {'PI_PASS':'login-secret'}), \
                patch('roboter_arm.provisioning.presentation.cli.load_connection', return_value=Connection()), \
                patch.object(subprocess, 'run') as run, patch.object(sys, 'stdout'), patch.object(sys, 'stderr'):
            run.return_value = subprocess.CompletedProcess([], 0, b'', b'')
            with self.assertRaises(SystemExit) as exit:
                main(['service', 'install', '--camera', 'oak'])
        self.assertEqual(exit.exception.code, 0)
        args, kwargs = run.call_args
        self.assertNotIn('login-secret', ' '.join(args[0]))
        password, unit = kwargs['input'].decode().split('\n', 1)
        self.assertEqual(password, 'login-secret')
        self.assertEqual(unit, service_unit(Connection(), camera='oak'))

    def test_camera_install_option_does_not_change_other_service_actions(self):
        with patch('roboter_arm.provisioning.presentation.cli.ssh') as remote, patch.object(sys, 'stderr'):
            for action in ('start', 'stop', 'restart', 'status'):
                with self.subTest(action=action), self.assertRaises(SystemExit) as exit:
                    main(['service', action, '--camera', 'oak'])
                self.assertEqual(exit.exception.code, 2)
            with self.assertRaises(SystemExit) as exit:
                main(['service', 'install', '--camera', 'unsupported'])
            self.assertEqual(exit.exception.code, 2)
            remote.assert_not_called()

    def test_service_install_sends_the_password_on_stdin_only(self):
        with patch.dict(os.environ, {'PI_PASS': 'login-secret'}), \
                patch('roboter_arm.provisioning.presentation.cli.load_connection', return_value=Connection()), \
                patch.object(subprocess, 'run') as run, patch.object(sys, 'stdout') as stdout, patch.object(sys, 'stderr'):
            run.return_value = subprocess.CompletedProcess([], 0, b'', b'')
            with self.assertRaises(SystemExit) as exit:
                main(['service', 'install'])
        self.assertEqual(exit.exception.code, 0)
        args, kwargs = run.call_args
        self.assertNotIn('login-secret', ' '.join(args[0]))
        password, unit = kwargs['input'].decode().split('\n', 1)
        self.assertEqual(password, 'login-secret')
        self.assertEqual(unit, service_unit(Connection()))
        self.assertIn(b'App: http://raspberrypi.local:8765/', stdout.buffer.write.call_args_list[0][0][0])
        with patch.dict(os.environ, {'PI_PASS': ''}), self.assertRaises(ValueError):
            install_payload(Connection())

    def test_install_enables_lingering_writes_the_unit_and_starts_it_after_sudo(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'log'
            # Like sudo -S -k -p '': read one password line, then run the command.
            (Path(directory) / 'sudo').write_text('#!/bin/sh\nIFS= read -r pw\n[ "$pw" = right ] || exit 1\nshift 4\nexec "$@"\n')
            for tool in ('loginctl', 'systemctl'):
                (Path(directory) / tool).write_text(f'#!/bin/sh\necho {tool} "$@" >> "{log}"\n')
            for tool in ('sudo', 'loginctl', 'systemctl'):
                (Path(directory) / tool).chmod(0o755)
            env = dict(os.environ, PATH=f'{directory}:{os.environ["PATH"]}', HOME=directory)
            unit = Path(directory) / '.config/systemd/user/robot-arm.service'
            result = subprocess.run(['sh', '-c', INSTALL_COMMAND], input='wrong\n[Unit]\n', text=True, env=env, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(unit.exists() or log.exists())
            for _ in range(2):
                result = subprocess.run(['sh', '-c', INSTALL_COMMAND], input='right\n[Unit]\nx=1\n', text=True, env=env, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(unit.read_text(), '[Unit]\nx=1\n')
            user = subprocess.run(['id', '-un'], capture_output=True, text=True).stdout.strip()
            self.assertEqual(log.read_text().splitlines()[:3], [f'loginctl enable-linger {user}', 'systemctl --user daemon-reload',
                                                                'systemctl --user enable --now robot-arm.service'])


class HostKeyTests(unittest.TestCase):
    fingerprint = 'SHA256:' + 'A' * 43
    key = b'raspberrypi.local ssh-ed25519 test-key'

    def connection(self, directory):
        return Connection(known_hosts=str(Path(directory) / 'known_hosts'))

    def results(self, fingerprint):
        return [subprocess.CompletedProcess([], 0, self.key+b'\n', b''),
                subprocess.CompletedProcess([], 0, f'256 {fingerprint} host (ED25519)\n'.encode(), b'')]

    def test_new_connection_requires_independent_fingerprint(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(subprocess, 'run') as run:
            with self.assertRaises(ValueError):
                ssh.setup_connection(self.connection(directory))
            run.assert_not_called()

    def test_mismatch_writes_nothing_and_does_not_authenticate(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(subprocess, 'run', side_effect=self.results('SHA256:'+'B'*43)), patch.object(ssh, 'ssh') as remote_ssh:
            with self.assertRaises(ValueError):
                ssh.setup_connection(self.connection(directory), self.fingerprint)
            self.assertFalse((Path(directory)/'known_hosts').exists())
            remote_ssh.assert_not_called()

    def test_matching_key_is_pinned_without_replacing_other_hosts(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(subprocess, 'run', side_effect=self.results(self.fingerprint)), patch.object(ssh, 'ssh') as remote_ssh:
            path = Path(directory)/'known_hosts'
            path.write_bytes(b'other.local ssh-ed25519 other-key\n')
            ssh.setup_connection(self.connection(directory), self.fingerprint)
            self.assertEqual(path.read_bytes(), b'other.local ssh-ed25519 other-key\n'+self.key+b'\n')
            remote_ssh.assert_called_once_with(self.connection(directory), 'python3 --version')

    def test_existing_different_key_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(subprocess, 'run', side_effect=self.results(self.fingerprint)), patch.object(ssh, 'ssh') as remote_ssh:
            path = Path(directory)/'known_hosts'
            previous = b'raspberrypi.local ssh-ed25519 old-key\n'
            path.write_bytes(previous)
            with self.assertRaises(ValueError):
                ssh.setup_connection(self.connection(directory), self.fingerprint)
            self.assertEqual(path.read_bytes(), previous)
            remote_ssh.assert_not_called()

    def test_duplicate_scan_results_for_same_key_are_accepted(self):
        results = self.results(self.fingerprint)
        results[0].stdout = self.key + b'\n' + self.key + b'\n'
        with tempfile.TemporaryDirectory() as directory, patch.object(subprocess, 'run', side_effect=results), patch.object(ssh, 'ssh'):
            ssh.setup_connection(self.connection(directory), self.fingerprint)
            self.assertEqual((Path(directory)/'known_hosts').read_bytes(), self.key+b'\n')


if __name__ == '__main__':
    unittest.main()
