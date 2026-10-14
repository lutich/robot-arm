"""The 3D view: its exported config stays in sync with the YAML, its JavaScript core passes
the node tests, and the sandbox server serves the page and its files."""
import shutil
import subprocess

import pytest

from armsim.config import ROOT
from armsim.export_web_config import TARGET, render, web_config


def test_exported_config_matches_the_yaml_files():
    current = TARGET.read_text()
    import json
    stored = json.loads(current.split('export const CONFIG = ', 1)[1].rstrip().rstrip(';'))
    fresh = web_config(oak=stored['oak'])
    fresh['robot_arm_commit'] = stored['robot_arm_commit']
    assert render(fresh) == current, 'run: python -m armsim.export_web_config'


@pytest.mark.skipif(shutil.which('node') is None, reason='node is not installed')
def test_javascript_core_passes_its_node_tests():
    result = subprocess.run(['node', '--test', str(ROOT / 'tests' / 'arm3d-core.test.js')], capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr


def test_view_files_exist_and_vendor_is_pinned():
    web = ROOT / 'web'
    for name in ('arm3d.html', 'arm3d.js', 'arm3d-core.js', 'arm3d-config.js', 'vendor/three.module.js', 'vendor/OrbitControls.js', 'vendor/RoomEnvironment.js'):
        assert (web / name).exists(), name
    assert "const REVISION = '170'" in (web / 'vendor' / 'three.module.js').read_text()[:400]
    html = (web / 'arm3d.html').read_text()
    assert '"three": "./vendor/three.module.js"' in html and 'arm3d.js' in html
