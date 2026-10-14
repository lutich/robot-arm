"""MujocoDriver under robot-arm's own Session: the driver contract and the session guards."""
import tempfile
from pathlib import Path

import pytest

from armsim.driver import MujocoDriver
from armsim.physics import SimClock
from armsim.robotarm import BOUNDS, HOME, PARK
from armsim.session import sandbox_session
from armsim.client import ArmError, SessionArmClient


@pytest.fixture
def world(physics):
    driver = MujocoDriver(physics)
    clock = SimClock(physics)
    with tempfile.TemporaryDirectory() as directory:
        session = sandbox_session(driver, directory=Path(directory) / 'poses', demo_directory=Path(directory) / 'demos',
                                  clock=clock.clock, sleep=clock.sleep)
        yield driver, session, SessionArmClient(session), physics
        session.close()


def test_construction_prepares_nothing(physics):
    driver = MujocoDriver(physics)
    assert not driver.prepared and all(not on for on in physics.enabled.values())


def test_write_disable_and_verify_off_follow_the_preview_driver_contract(physics):
    driver = MujocoDriver(physics)
    driver.prepare()
    driver.verify_off()
    driver.write(1, 427)
    assert driver.outputs[1] == 427 and physics.enabled[1] and physics.targets[1] == 427
    with pytest.raises(RuntimeError):
        driver.verify_off()
    for channel in range(16):      # Session.stop disables all sixteen PCA9685 channels
        driver.disable(channel)
    driver.verify_off()
    assert not physics.enabled[1]
    with pytest.raises(ValueError):
        driver.write(6, 400)
    with pytest.raises(ValueError):
        driver.write(1, 0)


def test_power_on_moves_park_to_home_and_the_physics_follows(world):
    driver, session, client, physics = world
    physics.place_arm(PARK)
    client.power_on()
    client.wait_idle()
    assert session.state()['commands'] == {c: HOME[c] for c in range(6)}
    assert driver.writes[0][1:] == (0, PARK[0])
    for channel in range(6):
        assert physics.joint_angle(channel) == pytest.approx(physics.calibration.angle(channel, HOME[channel]), abs=1.5)


def test_session_refuses_counts_outside_robot_arm_bounds(world):
    driver, session, client, physics = world
    physics.place_arm(PARK)
    client.power_on()
    client.wait_idle()
    before = list(driver.writes)
    for channel, (low, high) in BOUNDS.items():
        with pytest.raises(ArmError):
            client.move(channel, high + 1)
        with pytest.raises(ArmError):
            client.move(channel, low - 1)
    assert driver.writes == before


def test_single_joint_move_runs_at_twenty_counts_per_second(world):
    driver, session, client, physics = world
    physics.place_arm(PARK)
    client.power_on()
    client.wait_idle()
    start = physics.time
    client.move(1, HOME[1] + 100)        # lean back: the gripper rises, nothing is in the way
    client.wait_idle()
    assert physics.time - start == pytest.approx(1.5 * 100 / 20, abs=0.1)
    assert physics.joint_angle(1) == pytest.approx(physics.calibration.angle(1, HOME[1] + 100), abs=1.5)


def test_stop_disables_every_output_and_the_arm_goes_limp(world):
    driver, session, client, physics = world
    physics.place_arm(PARK)
    client.power_on()
    client.wait_idle()
    client.stop()
    assert not any(driver.outputs.values())
    assert session.state()['commands'] == {c: None for c in range(6)}
    held = physics.joint_angle(1)
    physics.step(1.0)
    assert abs(physics.joint_angle(1) - held) > 2   # gravity moves the unpowered shoulder


def test_move_pose_moves_all_joints_together_at_the_demo_speed(world):
    driver, session, client, physics = world
    physics.place_arm(PARK)
    client.power_on()
    client.wait_idle()
    target = dict(HOME)
    target[1] += 40
    target[2] -= 40
    start = physics.time
    client.move_pose(target, speed=40)
    assert session.state()['commands'] == target
    assert physics.time - start == pytest.approx(1.5 * 40 / 40, abs=0.1)
    bad = dict(target)
    bad[0] = 10
    with pytest.raises(ArmError):
        client.move_pose(bad)


def test_fallback_to_single_joint_moves_reaches_the_same_pose(world):
    driver, session, client, physics = world
    physics.place_arm(PARK)
    client.power_on()
    client.wait_idle()
    session.move_pose = None            # a server without the action
    delattr(type(session), 'move_pose') if False else None
    client._move_pose = lambda pose, speed: (_ for _ in ()).throw(ArmError('Unknown path'))
    target = dict(HOME)
    target[1] -= 20
    target[0] += 30
    client.move_pose(target)
    assert session.state()['commands'] == target
