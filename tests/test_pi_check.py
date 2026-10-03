"""Passive inspection reports deployed bytes and served assets, without arming."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.presentation import http_api as app
from roboter_arm.provisioning.domain.connection import Connection
from roboter_arm.provisioning.domain.report import report_valid
from roboter_arm.provisioning.infrastructure import deployment
from roboter_arm.provisioning.infrastructure.inspection import inspection


class InspectionTests(unittest.TestCase):
    def inspect(self, root, *, port=8765, check_app=False, tool_directory=None):
        payload = inspection(Connection(remote_dir=str(root), app_port=port), app=check_app)
        env = os.environ.copy()
        if tool_directory is not None:
            env['PATH'] = str(tool_directory) + os.pathsep + env.get('PATH', '')
        result = subprocess.run([sys.executable, '-'], input=payload, capture_output=True, env=env)
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        return json.loads(result.stdout)

    def test_missing_deployment_report_does_not_create_runtime_or_records(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)/'missing'
            report = self.inspect(root)
            self.assertFalse(root.exists())
            self.assertFalse(report['files_match'])
            self.assertFalse(report['runtime']['exists'])

    def test_app_gets_verify_assets_without_preparation(self):
        records = Path(self.enterContext(tempfile.TemporaryDirectory(prefix='robot-check-test-')))
        session = app.session_for(app.PreviewDriver(), list(range(6)), directory=records / 'poses',
                                  demo_directory=records / 'demos')
        server = app.Server(session, 0)
        server.start()
        try:
            report = self.inspect(app.ROOT, port=server.server_address[1], check_app=True)
            self.assertTrue(report['files_match'])
            self.assertTrue(report['app']['assets_match'])
            self.assertTrue(report['app']['reachable'])
            self.assertFalse(session.prepared)
            self.assertFalse(session.armed)
        finally:
            server.stop()
            session.close()

    def test_report_passes_only_with_matching_files_runtime_aarch64_and_i2c(self):
        good = dict(files_match=True, runtime=dict(valid=True), machine='aarch64', i2c_access=True,
                    app=dict(reachable=True, assets_match=True))
        self.assertTrue(report_valid(good, app=True))
        for change in (dict(files_match=False), dict(runtime=dict(valid=False)), dict(machine='x86_64'), dict(i2c_access=False)):
            with self.subTest(change=change):
                self.assertFalse(report_valid(good | change, app=False))
        for app in (dict(reachable=False, assets_match=True), dict(reachable=True, assets_match=False)):
            with self.subTest(app=app):
                self.assertTrue(report_valid(good | dict(app=app), app=False))
                self.assertFalse(report_valid(good | dict(app=app), app=True))

    def test_runtime_versions_and_dependency_health_are_required(self):
        packages = {}
        for line in (app.ROOT/'requirements/pi-py313.lock').read_text().splitlines():
            if line and not line.startswith('#'):
                name, version = line.split()[0].lower().replace('_','-').split('==')
                packages[name] = version
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = deployment.deployment(Connection(remote_dir=str(root)))
            result = subprocess.run([sys.executable, '-'], input=payload, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            python = root / '.venv-runtime/bin/python'
            python.parent.mkdir(parents=True)
            tools = root / 'tools'
            tools.mkdir()
            uv = tools / 'uv'
            def interpreter(versions, check_exit):
                record = json.dumps(dict(python=[3,13],packages=versions))
                python.write_text(f'#!{sys.executable}\nprint({record!r})\n')
                python.chmod(0o755)
                uv.write_text(f'#!{sys.executable}\nimport sys\nassert sys.argv[1:] == ["pip", "check", "--python", {str(python)!r}, "--no-python-downloads"]\nsys.exit({check_exit})\n')
                uv.chmod(0o755)
            interpreter(packages, 0)
            self.assertTrue(self.inspect(root, tool_directory=tools)['runtime']['valid'])
            interpreter(packages, 1)
            self.assertFalse(self.inspect(root, tool_directory=tools)['runtime']['valid'])
            interpreter(packages | {'adafruit-blinka':'different-version'}, 0)
            self.assertFalse(self.inspect(root, tool_directory=tools)['runtime']['valid'])


if __name__ == '__main__':
    unittest.main()
