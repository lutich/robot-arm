"""The generated MuJoCo model reproduces the measured geometry (docs/arm-dimensions.md)."""
import mujoco
import numpy as np
import pytest

from armsim.calibration import Calibration
from armsim.model_builder import build_xml, halfround_vertices

MM = 0.001


def straight_up(cal):
    counts = {c: cal.count(c, 0.0) for c in range(5)}
    counts[5] = cal.gripper['open_count']
    return counts


def test_model_compiles_with_six_actuators(physics):
    assert physics.model.nu == 6
    assert [physics.model.actuator(i).name for i in range(6)] == [f'act_S{i}' for i in range(1, 7)]


def test_joint_heights_in_the_straight_up_pose(physics, robot, scene):
    physics.place_arm(straight_up(physics.calibration))
    base = np.array([scene['base']['x'], scene['base']['y'], 0.0])
    links = robot['links']
    expected = {'s2': links['shoulder_height'], 's3': links['shoulder_height'] + links['upper_arm'],
                's4': links['shoulder_height'] + links['upper_arm'] + links['forearm'],
                'flange': links['shoulder_height'] + links['upper_arm'] + links['forearm'] + links['wrist']}
    for site, height in expected.items():
        pos = physics.site(site)
        assert np.linalg.norm(pos[:2] - base[:2]) < 1 * MM
        assert pos[2] == pytest.approx(height, abs=1 * MM)
    assert physics.site('flange')[2] == pytest.approx(0.394, abs=1 * MM)


@pytest.mark.parametrize('count,gap,length', [(650, 0.045, 0.100), (490, 0.0, 0.115)])
def test_gripper_gap_and_length_open_and_closed(physics, count, gap, length):
    physics.place_arm(straight_up(physics.calibration))
    physics.place_arm({5: count})
    assert physics.pad_gap() == pytest.approx(gap, abs=1 * MM)
    assert abs(physics.grasp_point()[2] - physics.site('flange')[2]) == pytest.approx(length, abs=1 * MM)   # gripper points up here


def test_gripper_holding_the_cube_matches_the_measured_height(physics):
    physics.place_arm(straight_up(physics.calibration))
    physics.place_arm({5: physics.calibration.count(5, 34.7)})
    assert physics.pad_gap() == pytest.approx(0.0265, abs=1 * MM)
    assert abs(physics.grasp_point()[2] - physics.site('flange')[2]) == pytest.approx(0.110, abs=1.5 * MM)


def test_home_pose_points_the_gripper_down(physics):
    from armsim.robotarm import HOME
    physics.place_arm(HOME)
    flange, grasp = physics.site('flange'), physics.grasp_point()
    tilt = np.degrees(np.arctan2(np.linalg.norm((flange - grasp)[:2]), flange[2] - grasp[2]))
    assert tilt < 16                     # the assumed HOME gripper is within 15 degrees of vertical
    assert np.linalg.norm(flange - grasp) == pytest.approx(0.100, abs=1 * MM)


def test_blocks_start_on_their_slots(physics, scene):
    poses = physics.block_poses()
    for block in scene['blocks']:
        slot = scene['slots'][scene['layout'][block['name']]]
        pos, yaw = poses[block['name']]
        assert pos[0] == pytest.approx(slot['x'], abs=1e-6) and pos[1] == pytest.approx(slot['y'], abs=1e-6)
        assert yaw == pytest.approx(slot.get('yaw', 0), abs=1e-6)


def test_halfround_mesh_matches_its_dimensions():
    vertices = np.array(halfround_vertices(0.060, 0.028, 0.012)).reshape(-1, 3)
    assert vertices[:, 0].min() == pytest.approx(-0.030) and vertices[:, 0].max() == pytest.approx(0.030)
    assert vertices[:, 1].min() == pytest.approx(-0.006) and vertices[:, 1].max() == pytest.approx(0.006)
    assert vertices[:, 2].min() == pytest.approx(0.0) and vertices[:, 2].max() == pytest.approx(0.028, abs=1e-6)


def test_arm_links_do_not_collide_with_each_other(robot, scene):
    model = mujoco.MjModel.from_xml_string(build_xml(robot, scene))
    data = mujoco.MjData(model)
    cal = Calibration(robot)
    from armsim.robotarm import PARK
    for c, count in PARK.items():
        data.qpos[model.jnt_qposadr[model.joint(f'S{c + 1}').id]] = cal.radians(c, count)
    mujoco.mj_forward(model, data)
    arm_geoms = {model.geom(i).name for i in range(model.ngeom) if model.geom_contype[i] == 2}
    for i in range(data.ncon):
        con = data.contact[i]
        names = {model.geom(con.geom1).name, model.geom(con.geom2).name}
        assert not names <= arm_geoms, f'arm self-contact {names}'
