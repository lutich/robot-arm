"""Bundled artifacts must exactly satisfy the tested dependency lock."""
import hashlib
import json
from pathlib import Path
import re
import tomllib
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]


class RuntimeBundleTests(unittest.TestCase):
    def test_complete_wheel_bundle_matches_lock_and_contains_metadata(self):
        lock = {}
        for line in (ROOT/'requirements/pi-py313.lock').read_text().splitlines():
            if line and not line.startswith('#'):
                package, digest = line.split()
                name, version = package.lower().replace('_','-').split('==')
                lock[name] = (version, digest.removeprefix('--hash=sha256:'))
        wheels = json.loads((ROOT/'runtime/wheels-manifest.json').read_text())
        self.assertEqual({wheel['name'].lower().replace('_','-') for wheel in wheels}, set(lock))
        self.assertEqual({wheel['file'] for wheel in wheels}, {p.name for p in (ROOT/'runtime/wheels').glob('*.whl')})
        for wheel in wheels:
            with self.subTest(wheel=wheel['file']):
                name = wheel['name'].lower().replace('_','-')
                self.assertEqual((wheel['version'],wheel['sha256']), lock[name])
                path = ROOT/'runtime/wheels'/wheel['file']
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),wheel['sha256'])
                with zipfile.ZipFile(path) as archive:
                    self.assertTrue(any(n.endswith('.dist-info/METADATA') for n in archive.namelist()))

    def test_laptop_lock_matches_the_pi_for_shared_packages(self):
        normalize = lambda name: re.sub(r'[-_.]+', '-', name).lower()
        pi = {}
        for line in (ROOT/'requirements/pi-py313.lock').read_text().splitlines():
            if line and not line.startswith('#'):
                name, version = line.split()[0].split('==')
                pi[normalize(name)] = version
        with (ROOT/'uv.lock').open('rb') as stream:
            laptop = {normalize(package['name']): package['version'] for package in tomllib.load(stream)['package']}
        shared = set(pi) & set(laptop)
        self.assertLessEqual({'fastapi', 'starlette', 'pydantic', 'pydantic-core', 'uvicorn'}, shared)
        self.assertEqual({name: laptop[name] for name in shared}, {name: pi[name] for name in shared})


if __name__ == '__main__':
    unittest.main()
