"""Inverse kinematics against analytic and MuJoCo forward kinematics."""
import itertools

import numpy as np
import pytest

from armsim.kinematics import Kinematics
from armsim.robotarm import BOUNDS, HOME

MM = 0.001


@pytest.fixture
def kin(robot, scene):
    return Kinematics(robot, scene, BOUNDS)


def test_forward_of_home_matches_mujoco(kin, physics):
    physics.place_arm(HOME)
    fk = kin.forward_counts(HOME)
    for site, key in (('s2', 's2'), ('s3', 's3'), ('s4', 's4'), ('flange', 'flange')):
        assert np.linalg.norm(physics.site(site) - fk[key]) < 1 * MM
    assert np.linalg.norm(physics.grasp_point() - fk['grasp']) < 1 * MM


def test_gripper_length_for_the_cube_gap(kin):
    assert kin.link_angle_for_gap(0.0265) == pytest.approx(34.7, abs=0.2)
    assert kin.gripper_length(gap=0.0265) == pytest.approx(0.110, abs=1.5 * MM)
    assert kin.gripper_length(gap=0.045) == pytest.approx(0.100, abs=1 * MM)
    assert kin.gripper_length(gap=0.0) == pytest.approx(0.115, abs=1 * MM)


def test_inverse_then_forward_hits_the_target(kin):
    for x, y, z in itertools.product((0.12, 0.17, 0.21), (-0.09, 0.0, 0.09), (0.013, 0.045)):
        angles = kin.inverse((x, y, z), gap=0.045)
        fk = kin.forward(angles)
        assert np.linalg.norm(fk['grasp'] - (x, y, z)) < 0.1 * MM
        assert fk['direction'][2] == pytest.approx(-1.0, abs=1e-6)      # pointing straight down
        assert angles[2] >= 0


def test_inverse_counts_then_mujoco_forward_hits_the_target_within_count_rounding(kin, physics):
    for x, y, z in itertools.product((0.14, 0.20), (-0.06, 0.06), (0.013, 0.05)):
        counts = kin.inverse_counts((x, y, z), gap=0.045)
        physics.place_arm(counts)
        assert np.linalg.norm(physics.grasp_point() - (x, y, z)) < 3 * MM


def test_unreachable_points_are_rejected(kin):
    with pytest.raises(ValueError):
        kin.inverse((0.5, 0.0, 0.013), gap=0.045)
    with pytest.raises(ValueError):
        kin.inverse_counts((0.33, 0.0, 0.013), gap=0.045)   # reachable only outside the count bounds


def test_block_yaw_turns_the_wrist_and_cubes_repeat_every_ninety_degrees(kin):
    straight = kin.inverse((0.18, 0.0, 0.013), gap=0.045, block_yaw=0)
    turned = kin.inverse((0.18, 0.0, 0.013), gap=0.045, block_yaw=30)
    same = kin.inverse((0.18, 0.0, 0.013), gap=0.045, block_yaw=120)
    assert turned[4] == pytest.approx(-30) and same[4] == pytest.approx(-30) and straight[4] == pytest.approx(0)


def test_long_blocks_repeat_every_180_degrees_and_stay_inside_the_servo_bounds(kin):
    counts = kin.inverse_counts((0.21, 0.05, 0.014), gap=0.012, block_yaw=90, symmetry=180)
    assert BOUNDS[4][0] <= counts[4] <= BOUNDS[4][1]
    angles = kin.calibration.angles(counts)
    offset = 90 if kin.g.get('pads_axis', 'x') == 'y' else 0
    pad_axis = angles[0] + offset - angles[4]             # direction along which the pads separate
    assert abs(((pad_axis - 0 + 90) % 180) - 90) < 1       # across the 12 mm depth of a block at yaw 90


def test_all_slots_are_reachable(kin, scene):
    for name, slot in scene['slots'].items():
        counts = kin.inverse_counts((slot['x'], slot['y'], 0.013), gap=0.045, block_yaw=slot.get('yaw', 0))
        assert set(counts) == set(range(6)), name
