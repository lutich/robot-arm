"""Grasp physics and the pick-and-place task, in-process with simulated time."""
import random
import tempfile
from pathlib import Path

import numpy as np
import pytest

from armsim.client import SessionArmClient
from armsim.driver import MujocoDriver
from armsim.physics import SimClock
from armsim.robotarm import BOUNDS, HOME, PARK
from armsim.session import sandbox_session
from armsim.tasks import PhysicsWorld, PickPlace

MM = 0.001
TARGETS = {'red': 'T1', 'green': 'T2', 'blue': 'T3', 'yellow': 'T4', 'roof': 'T5'}


@pytest.fixture
def world(physics, robot, scene):
    driver = MujocoDriver(physics)
    clock = SimClock(physics)
    with tempfile.TemporaryDirectory() as directory:
        session = sandbox_session(driver, directory=Path(directory) / 'poses', demo_directory=Path(directory) / 'demos',
                                  clock=clock.clock, sleep=clock.sleep)
        physics.place_arm(PARK)
        client = SessionArmClient(session)
        client.power_on()
        client.wait_idle()
        client.settings(demo_speed=100)
        task = PickPlace(client, PhysicsWorld(physics, scene), robot, scene, BOUNDS)
        yield task, client, physics, scene
        session.close()


def block_pose(physics, name):
    pos, yaw = physics.body_pose(name)
    return np.array(pos), yaw


def test_blocks_rest_without_drifting(world):
    task, client, physics, scene = world
    before = {name: block_pose(physics, name)[0] for name in physics.blocks}
    physics.step(5.0)
    for name, pos in before.items():
        assert np.linalg.norm(block_pose(physics, name)[0] - pos) < 1 * MM, name


def test_a_stack_of_four_cubes_stands_still(world):
    task, client, physics, scene = world
    size = 0.0265
    for level, name in enumerate(('red', 'green', 'blue', 'yellow')):
        physics.place_block(name, 0.25, -0.11, 0, z=size / 2 + level * size + 0.0005)
    physics.step(5.0)
    for level, name in enumerate(('red', 'green', 'blue', 'yellow')):
        pos, _ = block_pose(physics, name)
        assert abs(pos[0] - 0.25) < 1 * MM and abs(pos[1] + 0.11) < 1 * MM
        assert pos[2] == pytest.approx(size / 2 + level * size, abs=1 * MM)


def test_pick_lifts_the_block_without_slipping(world):
    task, client, physics, scene = world
    start, _ = block_pose(physics, 'red')
    task.pick('red')
    pos, yaw = block_pose(physics, 'red')
    assert pos[2] > start[2] + 0.03                       # lifted by about the safe height
    assert np.linalg.norm(physics.grasp_point() - pos) < 4 * MM
    assert physics.body_up('red') > 0.98
    held = physics.grasp_point() - pos
    carry = task.kin.inverse_counts((0.11, -0.11, pos[2]), 0.0265, 0)    # swing across the table at height
    carry[5] = client.commands()[5]
    client.move_pose(carry, speed=40)            # the task layer travels at this speed
    pos2, _ = block_pose(physics, 'red')
    held2 = physics.grasp_point() - pos2
    # The offset turns with the gripper, so compare its size and its vertical part, not its direction.
    assert abs(np.linalg.norm(held2) - np.linalg.norm(held)) < 2 * MM and abs(held2[2] - held[2]) < 2 * MM
    assert pos2[2] > 0.045 and pos2[1] < -0.09


@pytest.mark.parametrize('name', ['red', 'green', 'blue', 'yellow'])
def test_each_cube_goes_from_its_start_slot_to_its_target(world, name):
    task, client, physics, scene = world
    others = {other: block_pose(physics, other)[0] for other in physics.blocks if other != name}
    slot = scene['slots'][TARGETS[name]]
    task.run(name, TARGETS[name])
    physics.step(1.0)
    pos, yaw = block_pose(physics, name)
    assert np.linalg.norm(pos[:2] - (slot['x'], slot['y'])) < 5 * MM
    assert abs(((yaw - slot['yaw'] + 45) % 90) - 45) < 10
    assert physics.body_up(name) > 0.99 and pos[2] == pytest.approx(0.0265 / 2, abs=1 * MM)
    for other, before in others.items():
        assert np.linalg.norm(block_pose(physics, other)[0] - before) < 2 * MM, other
    assert client.commands() == HOME
    labels = [entry['label'] for entry in task.log]
    assert f'above {name}' in labels and labels[-1] == 'home'


def test_the_roof_block_is_gripped_across_its_depth(world):
    task, client, physics, scene = world
    slot = scene['slots']['T5']
    task.run('roof', 'T5')
    physics.step(1.0)
    pos, yaw = block_pose(physics, 'roof')
    assert np.linalg.norm(pos[:2] - (slot['x'], slot['y'])) < 5 * MM
    assert abs(((yaw - slot['yaw'] + 90) % 180) - 90) < 10
    assert physics.body_up('roof') > 0.99 and abs(pos[2]) < 2 * MM


def test_task_tolerates_small_calibration_offsets(world, monkeypatch):
    task, client, physics, scene = world
    rng = random.Random(7)
    offsets = {c: rng.randint(-5, 5) for c in range(6)}
    original = client._move_pose

    def skewed(pose, speed):
        return original({c: pose[c] + offsets[c] for c in pose}, speed)
    monkeypatch.setattr(client, '_move_pose', skewed)
    slot = scene['slots']['T2']
    task.run('green', 'T2')
    physics.step(1.0)
    pos, _ = block_pose(physics, 'green')
    assert np.linalg.norm(pos[:2] - (slot['x'], slot['y'])) < 12 * MM
    assert physics.body_up('green') > 0.99


def test_yellow_is_stacked_on_red(world):
    task, client, physics, scene = world
    red, _ = block_pose(physics, 'red')
    task.stack('yellow', 'red')
    physics.step(2.0)
    yellow, yaw = block_pose(physics, 'yellow')
    red2, _ = block_pose(physics, 'red')
    assert np.linalg.norm(yellow[:2] - red2[:2]) < 5 * MM
    assert yellow[2] == pytest.approx(0.0265 * 1.5, abs=1.5 * MM)
    assert np.linalg.norm(red2 - red) < 2 * MM
    assert physics.body_up('yellow') > 0.99 and abs(((yaw + 45) % 90) - 45) < 10
    assert client.commands() == HOME


def test_stacking_works_from_typed_block_positions(world, scene):
    """The real arm has no /sim/blocks: the operator types the positions, the physics checks them."""
    from armsim.tasks import ManualWorld
    task, client, physics, scene = world
    manual = ManualWorld(scene)
    manual.settle = lambda seconds=0.5: physics.step(seconds)
    for name in ('red', 'yellow'):
        slot = scene['slots'][scene['layout'][name]]
        manual.set(name, slot['x'], slot['y'], slot.get('yaw', 0))
    task.world = manual
    task.stack('yellow', 'red')
    physics.step(2.0)
    yellow, _ = block_pose(physics, 'yellow')
    red, _ = block_pose(physics, 'red')
    assert np.linalg.norm(yellow[:2] - red[:2]) < 5 * MM and yellow[2] == pytest.approx(0.0265 * 1.5, abs=1.5 * MM)
    assert manual.blocks()['yellow']['z'] == pytest.approx(0.0265 * 1.5, abs=1e-6)   # bookkeeping follows the move
