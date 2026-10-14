"""Demo records from the real arm and pose logs play in the sandbox."""
import json
import tempfile
from pathlib import Path

import pytest

from armsim.client import ArmError, SessionArmClient
from armsim.driver import MujocoDriver
from armsim.physics import SimClock
from armsim.replay import import_demo, read_demo_record, read_pose_log, replay_pose_log, run_demo
from armsim.robotarm import BOUNDS, DemoStore, HOME, PARK
from armsim.session import sandbox_session

POSITIONS = [dict(name='Lift', counts={0: 320, 1: 400, 2: 360, 3: 240, 4: 480, 5: 650}),
             dict(name='Turn', counts={0: 380, 1: 400, 2: 360, 3: 240, 4: 480, 5: 650}),
             dict(name='Grip', counts={0: 380, 1: 400, 2: 360, 3: 240, 4: 480, 5: 560})]


def commissioning_record(path, positions=POSITIONS, name='Pi demo'):
    record = dict(schema_version=1, recorded_at='2026-10-11T10:00:00+00:00', name=name,
                  positions=[dict(name=p['name'], counts={str(c): v for c, v in p['counts'].items()}) for p in positions],
                  scope='commissioning', units='PCA9685_counts', nominal_frequency_hz=60,
                  validated_limits=False, position_feedback=False)
    path.write_text(json.dumps(record))
    return path


@pytest.fixture
def world(physics):
    driver = MujocoDriver(physics)
    clock = SimClock(physics)
    with tempfile.TemporaryDirectory() as directory:
        demos = Path(directory) / 'demos'
        session = sandbox_session(driver, directory=Path(directory) / 'poses', demo_directory=demos,
                                  clock=clock.clock, sleep=clock.sleep)
        physics.place_arm(PARK)
        client = SessionArmClient(session)
        client.power_on()
        client.wait_idle()
        yield session, client, DemoStore(demos, 'synthetic', BOUNDS), Path(directory), physics
        session.close()


def test_commissioning_record_is_read_and_checked(tmp_path):
    record = read_demo_record(commissioning_record(tmp_path / 'demo.json'))
    assert record['scope'] == 'commissioning' and [p['name'] for p in record['positions']] == ['Lift', 'Turn', 'Grip']
    assert record['positions'][0]['counts'] == POSITIONS[0]['counts']
    bad = [dict(name='Out', counts={0: 320, 1: 400, 2: 360, 3: 240, 4: 480, 5: 700})]
    with pytest.raises(ValueError):
        read_demo_record(commissioning_record(tmp_path / 'bad.json', bad))
    with pytest.raises(ValueError):
        read_demo_record(tmp_path / 'missing.json')


def test_robot_arm_refuses_the_hardware_record_but_the_import_plays_it(world, tmp_path):
    session, client, store, directory, physics = world
    source = commissioning_record(tmp_path / 'demo.json')
    hardware_copy = directory / 'demos' / 'commissioning'
    hardware_copy.mkdir(parents=True)
    (hardware_copy / ('a' * 32 + '.json')).write_text(source.read_text())
    with pytest.raises(ArmError):
        client.demo('load', filename='a' * 32 + '.json')      # scope-locked, as on the Pi
    filename = import_demo(source, store)
    saved = session.state()['demo']['saved']
    assert any(d['filename'] == filename and d['positions'] == 3 and 'demo.json' in d['name'] for d in saved)
    run_demo(client, filename)
    state = session.state()
    assert state['commands'] == POSITIONS[-1]['counts']
    assert state['demo']['cursor'] == 3 and state['demo']['status'] == 'complete'
    for channel in range(5):
        assert physics.joint_angle(channel) == pytest.approx(physics.calibration.angle(channel, POSITIONS[-1]['counts'][channel]), abs=1.5)


def test_pose_log_replays_to_the_same_final_commands(world, tmp_path):
    session, client, store, directory, physics = world
    log = tmp_path / 'poses.jsonl'
    entries = [dict(label='a', counts=[320, 400, 380, 240, 480, 650], speed=40),
               dict(label='b', counts=[360, 420, 360, 250, 480, 560], speed=40)]
    log.write_text(''.join(json.dumps(e) + '\n' for e in entries))
    replay_pose_log(client, read_pose_log(log))
    assert session.state()['commands'] == dict(enumerate(entries[-1]['counts']))
    (tmp_path / 'empty.jsonl').write_text('')
    with pytest.raises(ValueError):
        read_pose_log(tmp_path / 'empty.jsonl')
