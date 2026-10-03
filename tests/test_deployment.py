"""Real temporary-directory payload execution; no SSH or hardware."""
import contextlib
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.provisioning.domain.connection import Connection
from roboter_arm.provisioning.infrastructure import deployment


class StateHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        payload = json.dumps(self.server.state).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


@contextlib.contextmanager
def app(state):
    """Stand-in for the running app's /api/state on the Pi."""
    server = ThreadingHTTPServer(('127.0.0.1', 0), StateHandler)
    server.state = state
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval':.01}, daemon=True)
    thread.start()
    try:
        yield server.server_address[1]
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class DeploymentTests(unittest.TestCase):
    def execute(self, root, files=None, baseline=None, *, port=None, env=None):
        connection = Connection(remote_dir=str(root), app_port=port or free_port())
        return subprocess.run([sys.executable, '-'], input=deployment.deployment(connection, files, baseline),
                              capture_output=True, env=env)

    def fake_systemctl(self, directory, active):
        """PATH with a systemctl that logs its arguments; is-active answers with the given status."""
        tools = Path(directory) / 'bin'
        tools.mkdir()
        log = Path(directory) / 'systemctl.log'
        (tools / 'systemctl').write_text(f'#!/bin/sh\necho "$@" >> "{log}"\n[ "$2" = is-active ] && exit {active}\nexit 0\n')
        (tools / 'systemctl').chmod(0o755)
        return dict(os.environ, PATH=f'{tools}:{os.environ["PATH"]}'), log

    def test_private_paths_and_unlisted_data_are_rejected(self):
        for files in ([], ['.env'], ['.pi.json'], ['artifacts/poses/local.json'],
                      ['../outside'], ['scripts/joint_check.py','scripts/joint_check.py']):
            with self.subTest(files=files), self.assertRaises(ValueError):
                deployment.entries(files)

    def test_every_pi_runtime_module_is_in_the_manifest(self):
        root = Path(__file__).resolve().parents[1]
        runtime = {'src/roboter_arm/__init__.py'} | {str(path.relative_to(root))
            for context in ('shared', 'control', 'calibration') for path in (root/'src/roboter_arm'/context).rglob('*.py')}
        self.assertEqual(sorted(runtime - set(deployment.manifest())), [])

    def test_full_manifest_deploys_and_retains_existing_operational_data(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'arm'
            saved = root / 'artifacts/demos/commissioning/existing.json'
            saved.parent.mkdir(parents=True)
            saved.write_text('Pi recording')
            result = self.execute(root)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertIn(b'App service not running', result.stdout)
            for entry in deployment.entries():
                self.assertEqual(hashlib.sha256((root / entry['path']).read_bytes()).hexdigest(), entry['sha256'])
            self.assertEqual(saved.read_text(), 'Pi recording')
            self.assertFalse((root / '.env').exists())
            result = self.execute(root)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertIn(b'already matches', result.stdout)

    def test_all_conflicts_checked_before_any_write_and_baseline_is_exact(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            changed = root / 'web/manual_control.css'
            changed.parent.mkdir()
            changed.write_text('Pi-side edit')
            files = ['scripts/joint_check.py', 'web/manual_control.css']
            result = self.execute(root, files)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((root/'scripts/joint_check.py').exists())
            digest = hashlib.sha256(changed.read_bytes()).hexdigest()
            self.assertNotEqual(self.execute(root, files, {'web/manual_control.css':'0'*64}).returncode, 0)
            self.assertEqual(self.execute(root, files, {'web/manual_control.css':digest}).returncode, 0)

    def test_files_unchanged_since_the_last_deploy_update_without_a_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'arm'
            files = ['web/manual_control.css', 'scripts/joint_check.py']
            self.assertEqual(self.execute(root, files).returncode, 0)
            expected = {entry['path']: entry['sha256'] for entry in deployment.entries(files)}
            self.assertEqual(json.loads((root/'.deployed.json').read_text()), expected)
            # The Pi still holds an older version that a previous deploy wrote.
            previous = b'previously deployed version'
            for name in files:
                (root/name).write_bytes(previous)
            (root/'.deployed.json').write_text(json.dumps({name: hashlib.sha256(previous).hexdigest() for name in files}))
            result = self.execute(root, files)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            for name, digest in expected.items():
                self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(), digest)

    def test_pi_side_edits_since_the_last_deploy_still_refuse(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'arm'
            changed = root / 'web/manual_control.css'
            self.assertEqual(self.execute(root, ['web/manual_control.css']).returncode, 0)
            for record in (None, '{corrupt'):
                with self.subTest(record=record):
                    changed.write_text('Pi-side edit')
                    if record is not None:
                        (root/'.deployed.json').write_text(record)
                    result = self.execute(root, ['web/manual_control.css'])
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(b'Pi-side change', result.stderr)
                    self.assertEqual(changed.read_text(), 'Pi-side edit')
            reference = root / 'config/reference-arm/home.json'
            reference.parent.mkdir(parents=True)
            reference.write_text('different evidence')
            (root/'.deployed.json').write_text(json.dumps({'config/reference-arm/home.json': hashlib.sha256(b'different evidence').hexdigest()}))
            result = self.execute(root, ['config/reference-arm/home.json'])
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(reference.read_text(), 'different evidence')

    def test_symlink_escape_and_reference_overwrite_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'arm'
            outside = Path(directory) / 'outside'
            outside.mkdir()
            root.mkdir()
            (root / 'scripts').symlink_to(outside, target_is_directory=True)
            result = self.execute(root, ['scripts/joint_check.py'])
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((outside/'joint_check.py').exists())
            reference = root / 'config/reference-arm/home.json'
            reference.parent.mkdir(parents=True)
            reference.write_text('different evidence')
            digest = hashlib.sha256(reference.read_bytes()).hexdigest()
            result = self.execute(root, ['config/reference-arm/home.json'], {'config/reference-arm/home.json':digest})
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(reference.read_text(), 'different evidence')

    def test_powered_arm_refuses_before_any_write(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'arm'
            files = ['web/manual_control.css']
            with app({'outputs_off': False}) as port:
                result = self.execute(root, files, port=port)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b'Arm is powered; press Power off before deploying', result.stderr)
            self.assertFalse(root.exists())
            with app({'outputs_off': True}) as port:
                self.assertEqual(self.execute(root, files, port=port).returncode, 0)

    def test_running_service_restarts_only_after_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'arm'
            files = ['web/manual_control.css']
            env, log = self.fake_systemctl(directory, active=0)
            with app({'outputs_off': True}) as port:
                result = self.execute(root, files, port=port, env=env)
                self.assertEqual(result.returncode, 0, result.stderr.decode())
                self.assertIn(b'App service restarted', result.stdout)
                self.assertEqual(log.read_text().splitlines(), ['--user is-active --quiet robot-arm.service',
                                                                '--user restart robot-arm.service'])
                log.unlink()
                result = self.execute(root, files, port=port, env=env)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertNotIn(b'restarted', result.stdout)
            self.assertEqual(log.read_text().splitlines(), ['--user is-active --quiet robot-arm.service'])

    def test_stopped_service_is_not_started_by_deploy(self):
        with tempfile.TemporaryDirectory() as directory:
            env, log = self.fake_systemctl(directory, active=3)
            result = self.execute(Path(directory) / 'arm', ['web/manual_control.css'], env=env)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertIn(b'App service not running', result.stdout)
            self.assertEqual(log.read_text().splitlines(), ['--user is-active --quiet robot-arm.service'])


if __name__ == '__main__':
    unittest.main()
