"""Arm simulation: kinematics, collision, cube world, and the driver wrapper behind the session."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.application.session import HOME, PARK
from roboter_arm.control.infrastructure.drivers import PreviewDriver
from roboter_arm.control.infrastructure.simulated_arm import SimulatedArm, SimulatedCollision
from roboter_arm.control.presentation.http_api import session_for
from roboter_arm.sim.domain.arm_model import BOARD, CUBE, SERVOS
from roboter_arm.sim.domain.collision import (
    Box, arm_shapes, box_box_sep, find_contacts, seg_box_dist, seg_seg_dist)
from roboter_arm.sim.domain.kinematics import (
    Unreachable, angles_from_counts, counts_for_gap, counts_from_angles, fk, gap_for_theta,
    gripper_theta, ik_top_down, max_reach, pad_depth_for_theta, rot_z, tool_yaw_and_tilt)
from roboter_arm.sim.domain.world import World
from test_motion import FakeTime

REF = [servo.ref for servo in SERVOS]
HOME_COUNTS = [HOME[c] for c in range(6)]


def top_down(x, y, zc, s6):
    return counts_from_angles(ik_top_down(x, y, zc), s6)


def drive(world, start, path, steps=20):
    """Step the world through straight count interpolations between waypoints; the last counts."""
    for target in path:
        for k in range(1, steps + 1):
            world.step([a + (b - a) * k / steps for a, b in zip(start, target)])
        start = target
    return start


class KinematicsTests(unittest.TestCase):
    """Reference values from px-robotic-arm-sim threejs-sim/test/core.test.mjs."""

    def test_ik_matches_the_pick_and_place_table(self):
        rows = [(5, 15, 0, 18.4, 17.9, 74.7, 87.4), (5, 15, 5, 18.4, 23.3, 44.4, 112.3),
                (-8, 12, 0, -33.7, 10.2, 84.0, 85.9), (0, 10, 0, 0, -14.2, 107.6, 86.6),
                (0, 20, 0, 0, 46.3, 33.8, 99.9)]
        for x, y, h, *expected in rows:
            with self.subTest(x=x, y=y, h=h):
                for got, want in zip(ik_top_down(x, y, CUBE / 2 + h), expected):
                    self.assertAlmostEqual(got, want, delta=0.06)
        with self.assertRaises(Unreachable):
            ik_top_down(0, 25, CUBE / 2)
        self.assertAlmostEqual(max_reach(), 21.1, delta=0.05)

    def test_fk_puts_the_held_cube_centre_on_the_ik_target_tool_down(self):
        for x, y, yaw in ((5, 15, 0), (-8, 12, 10), (0, 12, 30), (10, 10, -20), (-12, 6, 0)):
            with self.subTest(x=x, y=y, yaw=yaw):
                f = fk(ik_top_down(x, y, 4, cube_yaw=yaw, s5_near=90))
                for got, want in zip(f.grasp, (x, y, 4)):
                    self.assertAlmostEqual(got, want, places=9)
                jaw, tilt = tool_yaw_and_tilt(f.hand)
                self.assertAlmostEqual(tilt, 0, delta=1e-6)
                self.assertAlmostEqual(((jaw - yaw) % 90 + 135) % 90 - 45, 0, places=6)

    def test_gripper_linkage_reproduces_the_measured_values(self):
        self.assertAlmostEqual(gap_for_theta(56.4), 4.5, delta=0.01)
        self.assertAlmostEqual(pad_depth_for_theta(56.4), 10.0, delta=0.01)
        self.assertAlmostEqual(gap_for_theta(11.0), 0, delta=0.01)
        self.assertAlmostEqual(pad_depth_for_theta(11.0), 11.5, delta=0.01)
        self.assertAlmostEqual(gap_for_theta(gripper_theta(490)), 4.5, delta=0.01)
        self.assertTrue(490 < counts_for_gap(CUBE) < 590)

    def test_default_calibration_fits_the_taught_demo(self):
        f = fk(angles_from_counts([440, 347, 360, 185, 340, 490]))
        self.assertAlmostEqual(f.grasp[2], CUBE / 2, delta=0.5)
        self.assertLess(f.grasp[0], -5)
        self.assertLess(tool_yaw_and_tilt(f.hand)[1], 10)
        self.assertEqual(counts_from_angles(angles_from_counts(REF), REF[5]), REF)


class CollisionTests(unittest.TestCase):
    def test_primitives(self):
        self.assertAlmostEqual(seg_seg_dist((0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)), 1, places=12)
        self.assertAlmostEqual(seg_seg_dist((0, 0, 0), (2, 0, 0), (1, -1, 1), (1, 1, 1)), 1, places=12)
        A = Box((0, 0, 0), rot_z(0), (1, 1, 1))
        self.assertAlmostEqual(box_box_sep(A, Box((3, 0, 0), rot_z(0), (1, 1, 1))), 1, places=9)
        self.assertLess(box_box_sep(A, Box((1.5, 0, 0), rot_z(0.3), (1, 1, 1))), 0)
        self.assertAlmostEqual(seg_box_dist((2, -5, 0), (2, 5, 0), A), 1, places=6)
        self.assertLess(seg_box_dist((-5, 0, 0), (5, 0, 0), A), 0)

    def test_home_is_free_and_the_wrist_in_the_board_is_caught(self):
        self.assertEqual(World().step(HOME_COUNTS), [])
        contacts = find_contacts(arm_shapes(fk(ik_top_down(0, 15, -1.5)), 4.5), [], BOARD)
        self.assertIn('board', {k.b for k in contacts})
        self.assertTrue(all(k.pressing for k in contacts if k.b == 'board'))
        deep = find_contacts(arm_shapes(fk(ik_top_down(0, 15, -3.5)), 4.5), [], BOARD)
        self.assertTrue(any(not k.pressing for k in deep if k.b == 'board'))


class WorldTests(unittest.TestCase):
    def test_scene_validates_and_numbers_repeated_colours(self):
        world = World()
        world.set_scene([('blue', 0, 15, 0, 0), ('blue', 5, 15, 0, 0)])
        self.assertEqual([c.id for c in world.cubes], ['blue', 'blue2'])
        for cubes in ([('purple', 0, 15, 0, 0)], [('blue', 0, 80, 0, 0)], [('blue', float('nan'), 15, 0, 0)]):
            with self.subTest(cubes=cubes), self.assertRaises(ValueError):
                world.set_scene(cubes)

    def test_a_cube_without_support_drops_onto_the_board(self):
        world = World()
        world.set_scene([('orange', 0, 15, 2, 0)])
        self.assertAlmostEqual(world.cubes[0].p[2], CUBE / 2)

    def test_grasp_carry_and_stack(self):
        world = World()
        world.set_scene([('blue', 0, 15, 0, 0), ('orange', 6, 13, 0, 0)])
        start = drive(world, HOME_COUNTS, [
            top_down(0, 15, CUBE / 2 + 5, 490), top_down(0, 15, CUBE / 2, 490), top_down(0, 15, CUBE / 2, 600)])
        self.assertEqual(world.holding.id, 'blue')
        self.assertTrue(world.cube('blue').held)
        drive(world, start, [top_down(0, 15, CUBE / 2 + 6, 600), top_down(6, 13, CUBE * 1.5 + 4, 600),
                             top_down(6, 13, CUBE * 1.5, 600), top_down(6, 13, CUBE * 1.5, 490)])
        blue = world.cube('blue')
        self.assertIsNone(world.holding)
        self.assertFalse(blue.held)
        self.assertEqual(blue.json()['level'], 1)
        self.assertAlmostEqual(blue.p[0], 6, delta=0.3)
        self.assertEqual([e['kind'] for e in world.events if e['kind'] in ('grasp', 'release')], ['grasp', 'release'])

    def test_lifting_a_cube_drops_the_one_on_top(self):
        world = World()
        world.set_scene([('blue', 0, 15, 0, 0), ('green', 0, 15, 1, 0)])
        world.holding = None
        lower = world.cube('blue')
        lower.held = True
        lower.p[2] += 5
        world.settle()
        self.assertAlmostEqual(world.cube('green').p[2], CUBE / 2)


class SimulatedArmTests(unittest.TestCase):
    def setUp(self):
        self.driver = PreviewDriver()

    def home(self, arm):
        """PARK to HOME with all joints together, as power_on does."""
        for k in range(1, 51):
            for channel in range(6):
                arm.write(channel, round(PARK[channel] + (HOME[channel] - PARK[channel]) * k / 50))

    def test_preview_refuses_a_colliding_write_before_the_driver_sees_it(self):
        arm = SimulatedArm(self.driver, blocking=True, rest=PARK)
        self.home(arm)
        with self.assertRaises(SimulatedCollision):
            for count in range(427, 184, -2):        # S2 forward: the S6 servo hits the board
                arm.write(1, count)
        refused = count
        self.assertGreater(self.driver.outputs[1], refused)
        self.assertIn('S6 servo × board', arm.snapshot()['stopped_by'])
        self.assertEqual(arm.snapshot()['counts'][1], self.driver.outputs[1])

    def test_fingers_pressing_on_the_board_are_not_a_collision(self):
        arm = SimulatedArm(self.driver, blocking=True, rest=PARK)
        self.home(arm)
        for count in range(404, 501, 2):             # S3 down: the fingers press on the board
            arm.write(2, count)
        snapshot = arm.snapshot()
        self.assertEqual(self.driver.outputs[2], 500)
        self.assertTrue(snapshot['contacts'])
        self.assertTrue(all(k['pressing'] for k in snapshot['contacts']))
        self.assertIsNone(snapshot['stopped_by'])
        self.assertTrue(any(e['kind'] == 'press' for e in snapshot['events']))

    def test_hardware_only_reports_contacts(self):
        arm = SimulatedArm(self.driver, blocking=False, rest=PARK)
        self.home(arm)
        for count in range(426, 184, -2):
            arm.write(1, count)
        self.assertEqual(self.driver.outputs[1], 186)
        snapshot = arm.snapshot()
        self.assertEqual(snapshot['mode'], 'observing')
        self.assertIsNone(snapshot['stopped_by'])
        self.assertTrue(any('real arm not stopped' in e['text'] for e in snapshot['events']))

    def test_observing_follows_the_real_commands_and_never_refuses(self):
        arm = SimulatedArm(self.driver, blocking=False, rest=PARK)
        arm.observe([HOME[c] for c in range(6)], False)
        self.assertEqual(arm.snapshot()['counts'], HOME_COUNTS)
        arm.observe([None, 185, None, None, None, None], False)     # S6 servo into the board
        snapshot = arm.snapshot()
        self.assertEqual(snapshot['counts'][1], 185)
        self.assertEqual(snapshot['counts'][0], HOME[0])
        self.assertIsNone(snapshot['stopped_by'])
        self.assertTrue(any('real arm not stopped' in e['text'] for e in snapshot['events']))
        self.assertEqual(self.driver.outputs, {c:0 for c in range(16)})

    def test_going_to_park_is_resting_not_a_collision(self):
        arm = SimulatedArm(self.driver, blocking=True, rest=PARK)
        for start, end in ((PARK, HOME), (HOME, PARK)):       # like power_on, then Go PARK
            for k in range(1, 51):
                for channel in range(6):
                    arm.write(channel, round(start[channel] + (end[channel] - start[channel]) * k / 50))
        snapshot = arm.snapshot()
        self.assertTrue(snapshot['resting'])
        self.assertTrue(snapshot['contacts'])
        self.assertIsNone(snapshot['stopped_by'])

    def test_session_move_into_the_board_stops_with_outputs_off(self):
        time = FakeTime()
        with tempfile.TemporaryDirectory() as directory:
            arm = SimulatedArm(self.driver, blocking=True, rest=PARK)
            session = session_for(arm, list(range(6)), directory=Path(directory) / 'poses',
                                  demo_directory=Path(directory) / 'demos', clock=time.clock, sleep=time.sleep)
            try:
                session.power_on(park_confirmed=True)
                session.worker.join(timeout=5)
                self.assertEqual(session.state()['commands'][2], HOME[2])
                session.move(channel=1, target=185, first_clear=False)
                session.worker.join(timeout=5)
                state = session.state()
                self.assertTrue(state['outputs_off'])
                self.assertIn('SimulatedCollision', state['error'])
                self.assertIn('S6 servo × board', arm.snapshot()['stopped_by'])
            finally:
                session.close()


if __name__ == '__main__':
    unittest.main()
