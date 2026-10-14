"""The sandbox server end to end: robot-arm's HTTP API, the camera routes and the text interface."""
import io
import json
import tempfile
import time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen

import numpy as np
import pytest

from armsim.cli import Shell
from armsim.client import ArmError, HttpArmClient
from armsim.robotarm import HOME, PARK
from armsim.server import Sandbox
from armsim.tasks import HttpWorld, PickPlace


@pytest.fixture(scope='module')
def served():
    with tempfile.TemporaryDirectory() as directory:
        sandbox = Sandbox(artifacts=Path(directory))
        server = sandbox.start(port=0)
        base = f'http://127.0.0.1:{server.server_address[1]}'
        yield sandbox, base
        sandbox.stop()


def test_state_and_power_on_through_http(served):
    sandbox, base = served
    client = HttpArmClient(base)
    state = client.state()
    assert state['mode'] == 'preview' and not state['armed']
    client.power_on()
    state = client.wait_idle()
    assert state['armed'] and {int(k): v for k, v in state['commands'].items()} == HOME
    for channel in range(6):
        assert sandbox.physics.joint_angle(channel) == pytest.approx(sandbox.physics.calibration.angle(channel, HOME[channel]), abs=1.5)


def test_sandbox_routes_and_move_pose(served):
    sandbox, base = served
    client = HttpArmClient(base)
    blocks = client.sim('blocks')
    assert set(blocks) == {'red', 'green', 'blue', 'yellow', 'roof'}
    assert client.sim('state')['held'] is None
    client.settings(demo_speed=100)
    target = dict(HOME)
    target[1] += 30
    client.move_pose(target)
    assert client.commands() == target
    bad = dict(target)
    bad[5] = 700
    with pytest.raises(ArmError):
        client.move_pose(bad)
    with urlopen(f'{base}/sim/snapshot.png?camera=top', timeout=10) as response:
        assert response.read().startswith(b'\x89PNG')
    with pytest.raises(HTTPError):
        urlopen(f'{base}/sim/snapshot.png?camera=nope', timeout=10)
    assert client.sim('reset', {})['ok']


def test_camera_routes_serve_rendered_frames_like_the_pi(served):
    sandbox, base = served
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        with urlopen(f'{base}/api/camera/status', timeout=5) as response:
            status = json.load(response)
        if status.get('connected') and status.get('latest'):
            break
        time.sleep(0.2)
    assert status['enabled'] and status['source'] == 'mujoco'
    with urlopen(f'{base}/api/camera/snapshot.jpg', timeout=10) as response:
        jpeg = response.read()
        metadata = json.loads(response.headers['X-Camera-Frame'])
    assert jpeg.startswith(b'\xff\xd8') and metadata['width'] == 1280 and metadata['sequence'] >= 1
    with urlopen(f'{base}/api/camera/capture', timeout=10) as response:
        capture = json.load(response)
    assert capture['depth_png'] is None and capture['rgb_jpeg']


def test_pick_and_place_over_http_with_the_text_interface(served):
    sandbox, base = served
    out = io.StringIO()
    shell = Shell(base, out=out)
    shell.client.settings(demo_speed=100)
    shell.run_line('home')
    shell.run_line('blocks')
    shell.run_line('run red T1')
    shell.run_line('state')
    text = out.getvalue()
    assert 'red moved to T1' in text and 'red     x=' in text
    slot = sandbox.scene['slots']['T1']
    time.sleep(0.5)
    pos, _ = sandbox.physics.body_pose('red')
    assert np.linalg.norm(np.array(pos[:2]) - (slot['x'], slot['y'])) < 0.006
    assert shell.client.commands() == HOME
    log = Path(tempfile.mkdtemp()) / 'run.jsonl'
    shell.run_line(f'log {log}')
    assert log.exists() and 'poses written' in out.getvalue()
    shell.run_line('nonsense')
    assert 'unknown command' in out.getvalue()


def test_demo_import_and_playback_through_the_text_interface(served, tmp_path):
    sandbox, base = served
    record = dict(schema_version=1, recorded_at='2026-10-11T10:00:00+00:00', name='Wave',
                  positions=[dict(name='Up', counts={str(c): v for c, v in dict(HOME, **{}).items()}),
                             dict(name='Turn', counts={'0': 380, '1': 427, '2': 403, '3': 220, '4': 480, '5': 650})],
                  scope='commissioning', units='PCA9685_counts', nominal_frequency_hz=60,
                  validated_limits=False, position_feedback=False)
    source = tmp_path / 'wave.json'
    source.write_text(json.dumps(record))
    out = io.StringIO()
    shell = Shell(base, out=out)
    shell.demo_store = type(shell.demo_store)(sandbox.session.demo_store.directory.parent, 'synthetic', shell.demo_store.bounds)
    shell.run_line('speed 100')
    shell.run_line('home')
    shell.run_line(f'demo import {source}')
    shell.run_line('demo list')
    shell.run_line('demo load Wave')
    shell.run_line('demo run')
    text = out.getvalue()
    assert 'imported as' in text and 'Wave (imported from wave.json' in text and 'demo finished' in text
    assert shell.client.commands()[0] == 380
    assert shell.client.state()['demo']['status'] == 'complete'


def test_the_3d_view_is_served_with_its_scripts(served):
    sandbox, base = served
    for path, needle in (('/arm3d', b'<canvas id="view">'), ('/arm3d.js', b"import * as THREE from 'three'"),
                         ('/arm3d-core.js', b'export class World'), ('/arm3d-config.js', b'export const CONFIG'),
                         ('/vendor/three.module.js', b"REVISION = '170'"), ('/vendor/OrbitControls.js', b'OrbitControls'),
                         ('/vendor/RoomEnvironment.js', b'RoomEnvironment')):
        with urlopen(f'{base}{path}', timeout=10) as response:
            body = response.read()
            assert response.status == 200 and needle in body, path
            assert 'javascript' in response.headers['Content-Type'] or path == '/arm3d'


def test_web_proxy_forwards_api_calls_to_the_arm(served):
    import threading
    from armsim.web import serve
    sandbox, base = served
    proxy = serve(0, '127.0.0.1', base)
    thread = threading.Thread(target=proxy.serve_forever, daemon=True)
    thread.start()
    try:
        via = f'http://127.0.0.1:{proxy.server_address[1]}'
        with urlopen(f'{via}/arm3d', timeout=10) as response:
            assert b'<canvas id="view">' in response.read()
        with urlopen(f'{via}/api/state', timeout=10) as response:
            assert json.load(response)['mode'] == 'preview'
        client = HttpArmClient(via)                       # the same client, through the proxy
        client.settings(demo_speed=50)
        assert client.state()['settings']['demo_speed'] == 50
        with pytest.raises(ArmError):
            client.settings(demo_speed=500)               # refusals pass through with their message
        with urlopen(f'{via}/sim/blocks', timeout=10) as response:
            assert 'red' in json.load(response)
    finally:
        proxy.shutdown()
        proxy.server_close()


def test_shell_refuses_power_on_against_hardware_without_confirmation(served):
    sandbox, base = served
    out = io.StringIO()
    shell = Shell(base, out=out)
    real_state = shell.client.state
    shell.client.state = lambda: dict(real_state(), mode='hardware')
    calls = []
    shell.client.power_on = lambda: calls.append('power_on')
    shell.run_line('power on')
    assert calls == [] and 'power on confirmed' in out.getvalue()
    shell.run_line('power on confirmed')
    assert calls == ['power_on']


def test_shell_uses_typed_block_positions_when_the_target_has_none(served):
    sandbox, base = served
    out = io.StringIO()
    shell = Shell(base, out=out)
    shell.task.world.http.blocks = lambda: (_ for _ in ()).throw(ArmError('Unknown path'))
    shell.run_line('blocks')
    assert 'blocks set' in out.getvalue()
    shell.run_line('blocks set red 0.110 0.110')
    shell.run_line('blocks')
    assert 'red     x=0.110 y=0.110 z=0.013' in out.getvalue()



def _post(base, path, body, expect=200):
    from urllib.request import Request
    request = Request(f'{base}{path}', data=json.dumps(body).encode(), method='POST', headers={'Content-Type': 'application/json', 'Origin': base})
    try:
        with urlopen(request, timeout=10) as response:
            assert response.status == expect
            return json.load(response)
    except HTTPError as error:
        assert error.code == expect, error.read()
        raise ArmError(json.load(error).get('error'))


def _wait_move(base, timeout=120):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with urlopen(f'{base}/move_to/status', timeout=10) as response:
            status = json.load(response)
        if not status['running']:
            return status
        time.sleep(0.5)
    raise AssertionError('move did not finish')


def test_move_to_lowers_the_open_gripper_at_the_requested_point(served):
    sandbox, base = served
    client = HttpArmClient(base)
    if not client.state()['armed']:
        client.power_on()
        client.wait_idle()
    client.sim('reset', {})
    before = client.commands()
    with pytest.raises(ArmError):                                   # outside the table
        _post(base, '/move_to', dict(x=0.5, y=0.0), expect=400)
    with pytest.raises(ArmError):                                   # out of reach
        _post(base, '/move_to', dict(x=0.33, y=0.0), expect=400)
    assert client.commands() == before
    _post(base, '/move_to', dict(x=0.19, y=-0.02, z=0.02, path='safe'))
    status = _wait_move(base)
    assert status['done'] and status['error'] is None, status
    assert [e['label'] for e in status['log']][-2:] == ['travel above target', 'lower']
    time.sleep(0.5)
    grasp = np.array(sandbox.physics.grasp_point())
    assert np.linalg.norm(grasp - (0.19, -0.02, 0.02)) < 0.004
    assert sandbox.physics.pad_gap() == pytest.approx(0.045, abs=0.002)


def test_proxy_serves_move_to_for_a_target_without_it(served):
    import threading
    from armsim.web import Handler, serve
    sandbox, base = served
    client = HttpArmClient(base)
    if not client.state()['armed']:
        client.power_on()
        client.wait_idle()
    proxy = serve(0, '127.0.0.1', base)
    threading.Thread(target=proxy.serve_forever, daemon=True).start()
    try:
        via = f'http://127.0.0.1:{proxy.server_address[1]}'
        _post(via, '/move_to', dict(x=0.15, y=0.08, z=0.03, lower=False))          # direct by default
        status = _wait_move(via)
        assert status['done'] and status['error'] is None, status
        assert [e['label'] for e in status['log']] == ['go above target']
        time.sleep(0.5)
        grasp = np.array(sandbox.physics.grasp_point())
        assert np.linalg.norm(grasp[:2] - (0.15, 0.08)) < 0.004 and grasp[2] > 0.06
    finally:
        proxy.shutdown()
        proxy.server_close()
        for cls in Handler.__subclasses__():
            cls.mover = None


def test_move_to_dry_run_returns_the_poses_without_moving(served):
    sandbox, base = served
    client = HttpArmClient(base)
    if not client.state()['armed']:
        client.power_on()
        client.wait_idle()
    before = client.commands()
    result = _post(base, '/move_to', dict(x=0.17, y=0.05, z=0.02, dry=True))['result']
    assert [s['label'] for s in result['steps']] == ['go to target']
    safe = _post(base, '/move_to', dict(x=0.17, y=0.05, z=0.02, dry=True, path='safe'))['result']
    assert [s['label'] for s in safe['steps']][-2:] == ['travel above target', 'lower']
    assert all(len(s['counts']) == 6 for s in result['steps'])
    time.sleep(0.3)
    assert client.commands() == before
