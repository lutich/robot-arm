"""Exercise camera installer admission without downloads or hardware access."""
from pathlib import Path
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CameraRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        for name in ('scripts', 'requirements', 'bin', 'target/bin', '.venv-runtime/bin'):
            (self.root/name).mkdir(parents=True)
        for name in ('scripts/install_camera_runtime.sh', 'requirements/camera-py313.lock'):
            shutil.copyfile(ROOT/name, self.root/name)
        self.log = self.root/'calls'
        self.marker = self.root/'installed'
        quoted_log = shlex.quote(str(self.log))
        (self.root/'scripts/install_runtime.sh').write_text(f'#!/bin/sh\necho verified >> {quoted_log}\n')
        uv = self.root/'bin/uv'
        uv.write_text(f'#!/bin/sh\necho "uv:$*" >> {quoted_log}\n'
                      f'[ "$1:$2" != "pip:install" ] || touch {shlex.quote(str(self.marker))}\n')
        uv.chmod(0o755)
        interpreter = self.root/'target/bin/python'
        interpreter.write_text(f'''#!{sys.executable}
import importlib.metadata as metadata, os, pathlib, platform, sys, sysconfig
platform.system = lambda: os.environ.get('CAMERA_TEST_SYSTEM', 'Linux')
platform.machine = lambda: 'aarch64'
platform.python_implementation = lambda: 'CPython'
sys.version_info = (3, 13, 0)
sysconfig.get_config_var = lambda name: False
os.confstr = lambda name: os.environ.get('CAMERA_TEST_GLIBC', 'glibc 2.36')
def version(name):
    if pathlib.Path({str(self.marker)!r}).exists():
        return {{'numpy': '2.4.4', 'depthai': '3.6.1'}}[name]
    existing = os.environ.get({{'numpy': 'CAMERA_TEST_NUMPY', 'depthai': 'CAMERA_TEST_DEPTHAI'}}[name])
    if existing:
        return existing
    raise metadata.PackageNotFoundError(name)
metadata.version = version
sys.argv = sys.argv[1:]
exec(sys.stdin.read())
''')
        interpreter.chmod(0o755)
        shutil.copyfile(interpreter, self.root/'.venv-runtime/bin/python')
        (self.root/'.venv-runtime/bin/python').chmod(0o755)

    def run_installer(self, *arguments, default_target=False, **options):
        env = {**os.environ, 'PATH': str(self.root/'bin') + os.pathsep + os.environ['PATH'], **options}
        command = ['sh', str(self.root/'scripts/install_camera_runtime.sh')]
        if not default_target:
            command.append(str(self.root/'target'))
        return subprocess.run([*command, *arguments], env=env, capture_output=True, text=True)

    def test_platform_and_libc_refusals_do_not_install_or_verify(self):
        for options in ({'CAMERA_TEST_SYSTEM': 'Darwin'}, {'CAMERA_TEST_GLIBC': 'glibc 2.27'}):
            with self.subTest(options=options):
                result = self.run_installer(**options)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.log.exists())
                self.assertFalse(self.marker.exists())

    def test_existing_different_camera_dependency_is_not_replaced(self):
        result = self.run_installer(CAMERA_TEST_NUMPY='2.3.0')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('will not replace it', result.stderr)
        self.assertFalse(self.marker.exists())
        self.assertFalse(self.log.exists())

    def test_accepted_install_verifies_runtime_before_and_after(self):
        result = self.run_installer()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.log.read_text().splitlines()
        self.assertEqual(calls[0], 'verified')
        self.assertEqual(calls[-1], 'verified')
        self.assertIn('--require-hashes --no-deps', calls[1])
        self.assertIn('--only-binary=:all:', calls[1])
        self.assertTrue(calls[2].startswith('uv:pip check '))
        self.assertIn('no camera or controller was initialized', result.stdout)

    def test_known_sdk_replacement_requires_explicit_flag(self):
        result = self.run_installer(CAMERA_TEST_DEPTHAI='3.10.0', CAMERA_TEST_NUMPY='2.4.4')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('will not replace it', result.stderr)
        self.assertFalse(self.marker.exists())
        self.assertFalse(self.log.exists())

    def test_explicit_known_sdk_replacement_preserves_verification_and_install_guards(self):
        result = self.run_installer('--replace-camera-sdk', CAMERA_TEST_DEPTHAI='3.10.0',
                                    CAMERA_TEST_NUMPY='2.4.4')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.log.read_text().splitlines()
        self.assertEqual(len(calls), 4)
        self.assertEqual(calls[0], 'verified')
        self.assertEqual(calls[-1], 'verified')
        self.assertIn('--python ' + str(self.root/'target/bin/python'), calls[1])
        self.assertIn('--require-hashes --no-deps', calls[1])
        self.assertIn('--only-binary=:all:', calls[1])
        self.assertTrue(calls[2].startswith('uv:pip check '))
        self.assertIn('Pinned camera add-on verified', result.stdout)

    def test_replacement_flag_uses_default_target_without_positional_argument(self):
        result = self.run_installer('--replace-camera-sdk', default_target=True,
                                    CAMERA_TEST_DEPTHAI='3.10.0', CAMERA_TEST_NUMPY='2.4.4')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--python ' + str(self.root/'.venv-runtime/bin/python'), self.log.read_text())

    def test_matching_camera_versions_do_not_need_replacement_flag(self):
        result = self.run_installer(CAMERA_TEST_DEPTHAI='3.6.1', CAMERA_TEST_NUMPY='2.4.4')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Pinned camera add-on verified', result.stdout)

    def test_flag_does_not_allow_unknown_sdk_or_different_numpy(self):
        for options in ({'CAMERA_TEST_DEPTHAI':'3.9.0'}, {'CAMERA_TEST_DEPTHAI':'3.10.1'},
                        {'CAMERA_TEST_DEPTHAI':'3.10.0', 'CAMERA_TEST_NUMPY':'2.3.0'}):
            with self.subTest(options=options):
                result = self.run_installer('--replace-camera-sdk', **options)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('will not replace it', result.stderr)
                self.assertFalse(self.marker.exists())
                self.assertFalse(self.log.exists())

    def test_flag_does_not_allow_replacement_to_other_locked_version(self):
        lock = self.root/'requirements/camera-py313.lock'
        lock.write_text(lock.read_text().replace('depthai==3.6.1', 'depthai==3.9.0'))
        result = self.run_installer('--replace-camera-sdk', CAMERA_TEST_DEPTHAI='3.10.0')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('will not replace it', result.stderr)
        self.assertFalse(self.marker.exists())
        self.assertFalse(self.log.exists())

    def test_invalid_arguments_refuse_before_runtime_actions(self):
        for arguments in (('--unknown-camera',), ('--replace-camera-sdk', '--replace-camera-sdk'),
                          ('second-target',), ('',), ('--replace-camera-sdk', 'second-target')):
            with self.subTest(arguments=arguments):
                result = self.run_installer(*arguments)
                self.assertEqual(result.returncode, 2)
                self.assertIn('Usage:', result.stderr)
                self.assertFalse(self.marker.exists())
                self.assertFalse(self.log.exists())


if __name__ == '__main__':
    unittest.main()
