"""Counts to angles and back, checked against the measured 90° moves."""
import pytest

from armsim.calibration import Calibration
from armsim.robotarm import HOME, BOUNDS

# From docs/servo-calibration.md: (channel, from, to, degrees moved)
MEASURED = [(1, 467, 247, 90), (2, 543, 303, 90), (3, 483, 233, 90), (4, 250, 480, 90), (0, 320, 60, 85)]


@pytest.fixture
def cal(robot):
    return Calibration(robot)


def test_home_counts_match_robot_arm_home(cal):
    assert cal.home_counts == HOME


def test_home_angles_are_the_configured_reference(cal, robot):
    for channel in range(5):
        assert cal.angle(channel, HOME[channel]) == pytest.approx(robot['channels'][channel]['home_angle'])


def test_round_trip_is_exact_for_every_count_in_bounds(cal):
    for channel in range(6):
        low, high = BOUNDS[channel]
        for count in range(low, high + 1):
            assert cal.count(channel, cal.angle(channel, count)) == count


@pytest.mark.parametrize('channel,start,end,degrees', MEASURED)
def test_measured_moves_span_the_measured_angle(cal, channel, start, end, degrees):
    moved = abs(cal.angle(channel, end) - cal.angle(channel, start))
    assert moved == pytest.approx(degrees, abs=0.5)


def test_gripper_maps_open_and_closed_counts_to_link_angles(cal, robot):
    g = robot['gripper']
    assert cal.angle(5, g['open_count']) == pytest.approx(g['open_angle'])
    assert cal.angle(5, g['closed_count']) == pytest.approx(g['closed_angle'])
    assert cal.count(5, g['open_angle']) == g['open_count']


def test_block_grip_count_sits_between_open_and_closed(cal):
    # The team measured 580 counts as the block grip; the fitted linkage holds a 26,5 mm cube at 34,7°.
    assert 30 < cal.angle(5, 580) < 40
