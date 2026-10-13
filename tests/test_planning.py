"""Trajectory planning: parameters, task text, waypoints, controller steps, rehearsal and the mock's planner."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.application.session import HOME, PARK
from roboter_arm.control.infrastructure import trajectory_planner
from roboter_arm.control.infrastructure.drivers import PreviewDriver
from roboter_arm.control.infrastructure.simulated_arm import SimulatedArm
from roboter_arm.control.infrastructure.trajectory_planner import TrajectoryPlanner
from roboter_arm.sim.domain.arm_model import CUBE, SERVOS
from roboter_arm.sim.domain.planning.parameters import PLANNING_JSON, RULES, resolve, rules, scene_entries
from roboter_arm.sim.domain.planning.planner import guard_violations, plan_waypoints
from roboter_arm.sim.domain.planning.rehearsal import rehearse
from roboter_arm.sim.domain.planning.steps import PEAK, coordinated_steps, position, single_servo_steps, timed_steps
from roboter_arm.sim.domain.planning.task import PlanError, Task, parse_task
from roboter_arm.sim.domain.world import World

DEFAULTS = json.loads(PLANNING_JSON.read_text())
PARAMS = resolve(DEFAULTS)
PARK_COUNTS = [PARK[c] for c in range(6)]
HOME_COUNTS = [HOME[c] for c in range(6)]


def scene():
    world = World()
    world.set_scene(scene_entries(DEFAULTS['scene']))
    return world


def run(text, *, params=PARAMS, start=PARK_COUNTS, home=HOME_COUNTS, world=None):
    """Plan and rehearse; (waypoints, steps, rehearsal)."""
    world = world or scene()
    waypoints, expect = plan_waypoints(world, parse_task(text, world), start, params, home=home)
    steps = timed_steps(start, waypoints, params['motion'])
    return waypoints, steps, rehearse(world, start, steps, expect, params, rest=PARK_COUNTS)


def with_overrides(**groups):
    return resolve(DEFAULTS, groups)


class ParameterTests(unittest.TestCase):
    def test_planning_json_is_complete_and_valid(self):
        self.assertEqual(set(PARAMS), set(RULES))
        self.assertEqual(PARAMS['motion'], {'mode': 'single_servo', 'speed': 20, 'step_counts': 8})
        self.assertEqual(len(scene().cubes), 4)

    def test_overrides_replace_single_values_and_are_validated(self):
        params = with_overrides(motion={'speed': 50}, acceptance={'fail_on_contact': True})
        self.assertEqual((params['motion']['speed'], params['motion']['step_counts']), (50, 8))
        self.assertTrue(params['acceptance']['fail_on_contact'])
        for overrides in ({'physics': {}}, {'motion': {'warp': 1}}, {'motion': {'speed': 0}}, {'motion': {'speed': True}},
                          {'motion': {'mode': 'teleport'}}, {'motion': {'step_counts': 8.5}},
                          {'approach': {'hover_fallbacks': [1.0] * 9}}, {'gripper': {'hold': SERVOS[5].closed + 1}},
                          {'acceptance': {'fail_on_contact': 1}}, {'rehearsal': {'frame_seconds': float('nan')}}, ['motion']):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                resolve(DEFAULTS, overrides)

    def test_a_missing_default_is_named(self):
        broken = json.loads(json.dumps(DEFAULTS))
        del broken['approach']['hover']
        with self.assertRaisesRegex(ValueError, 'approach.hover is missing'):
            resolve(broken)

    def test_rules_describe_every_parameter_for_the_panel_form(self):
        described = rules()
        self.assertEqual(len(described), sum(len(keys) for keys in RULES.values()))
        self.assertEqual(described['motion.mode'], {'kind': 'choice', 'choices': ['single_servo', 'coordinated']})
        self.assertEqual(described['gripper.hold'], {'kind': 'integer', 'min': SERVOS[5].open, 'max': SERVOS[5].closed})

    def test_scene_entries_need_a_colour_and_a_position(self):
        self.assertEqual(scene_entries([{'colour': 'red', 'x': 1, 'y': 12}]), [('red', 1, 12, 0, 0)])
        for cubes in ({'colour': 'red'}, [{'colour': 'red', 'x': 1}], [{'colour': 'red', 'x': 1, 'y': 2, 'z': 0}],
                      [{'colour': 'red', 'x': '1', 'y': 2}], [{'colour': 'red', 'x': 1, 'y': 2, 'level': 1.5}],
                      [{'colour': 'red', 'x': 1, 'y': 12}] * 13):
            with self.subTest(cubes=cubes), self.assertRaises(ValueError):
                scene_entries(cubes)


class TaskTests(unittest.TestCase):
    def test_grammar(self):
        world = scene()
        for text, task in (('blue on orange', Task('blue', on='orange')), ('blau auf orange', Task('blue', on='orange')),
                           ('green to 5 15', Task('green', at=(5.0, 15.0))), ('move green to (5, 15)', Task('green', at=(5.0, 15.0))),
                           ('gelb nach -8 12', None), ('grab red', Task('red')), ('pick up red', Task('red')),
                           ('pick up orange block and bring it to home position', Task('orange', home=True)),
                           ('red on blue, then home', Task('red', on='blue', home=True)), ('home', Task(None, home=True))):
            with self.subTest(text=text):
                if task is None:
                    with self.assertRaisesRegex(PlanError, 'no yellow cube'):
                        parse_task(text, world)
                else:
                    self.assertEqual(parse_task(text, world), task)
        with self.assertRaisesRegex(PlanError, 'say'):
            parse_task('dance', world)


class PlannerTests(unittest.TestCase):
    def test_pick_and_place_waypoints_open_descend_grip_lift_and_release(self):
        world = scene()
        waypoints, expect = plan_waypoints(world, parse_task('green to 5 15', world), PARK_COUNTS, PARAMS)
        labels = [w['label'] for w in waypoints]
        self.assertEqual(labels[:3], ['Open gripper', 'Above green', 'Descend 1/5'])
        self.assertIn('Grip green', labels)
        self.assertEqual(labels[-1], 'Withdraw 5/5')
        grip = labels.index('Grip green')
        self.assertEqual(waypoints[grip]['counts'][5], PARAMS['gripper']['hold'])
        self.assertEqual(waypoints[grip]['counts'][:5], waypoints[grip - 1]['counts'][:5])
        self.assertEqual(waypoints[labels.index('Release')]['counts'][5], SERVOS[5].open)
        self.assertTrue(all(not guard_violations(w['counts']) for w in waypoints))
        self.assertEqual((expect['type'], expect['on'], expect['target']), ('place', 'board', [5.0, 15.0, CUBE / 2]))

    def test_a_cube_goes_onto_the_top_of_the_target_stack(self):
        world = scene()
        world.set_scene(scene_entries(DEFAULTS['scene']) + [('yellow', 3.0, 10.0, 1, 0)])
        _, expect = plan_waypoints(world, parse_task('blue on red', world), PARK_COUNTS, PARAMS)
        self.assertEqual(expect['on'], 'yellow')
        self.assertAlmostEqual(expect['target'][2], CUBE / 2 + 2 * CUBE)

    def test_impossible_tasks_say_why(self):
        world = scene()
        world.set_scene(scene_entries(DEFAULTS['scene']) + [('yellow', -8.0, 12.0, 1, 0)])
        for text, reason in (('blue on red', 'blue has yellow on top'), ('red on red', 'on itself'),
                             ('red to 14 30', 'unreachable'), ('red to 40 10', 'off the board'),
                             ('red to 12.8 14', 'on the edge of orange')):
            with self.subTest(text=text), self.assertRaisesRegex(PlanError, reason):
                plan_waypoints(world, parse_task(text, world), PARK_COUNTS, PARAMS)

    def test_home_ends_at_home_and_a_held_cube_stays_gripped(self):
        world = scene()
        waypoints, _ = plan_waypoints(world, parse_task('grab red home', world), PARK_COUNTS, PARAMS, home=HOME_COUNTS)
        self.assertEqual(waypoints[-1]['counts'], HOME_COUNTS[:5] + [PARAMS['gripper']['hold']])
        waypoints, _ = plan_waypoints(world, parse_task('red on blue home', world), PARK_COUNTS, PARAMS, home=HOME_COUNTS)
        self.assertEqual(waypoints[-1]['counts'], HOME_COUNTS)
        with self.assertRaisesRegex(PlanError, 'no HOME pose'):
            plan_waypoints(world, parse_task('grab red home', world), PARK_COUNTS, PARAMS)


class StepTests(unittest.TestCase):
    WAYPOINTS = [{'label': 'A', 'phase': 'pick', 'counts': [320, 427, 403, 220, 480, 490]},
                 {'label': 'B', 'phase': 'pick', 'counts': [301, 427, 380, 220, 480, 590]}]

    def test_single_servo_steps_move_one_servo_at_most_step_counts_and_reach_every_waypoint(self):
        for step_counts in (1, 3, 8, 50):
            with self.subTest(step_counts=step_counts):
                steps = single_servo_steps(PARK_COUNTS, self.WAYPOINTS, step_counts)
                previous = PARK_COUNTS
                for step in steps:
                    changed = [abs(a - b) for a, b in zip(previous, step['counts']) if a != b]
                    self.assertEqual(len(changed), 1)
                    self.assertLessEqual(changed[0], step_counts)
                    previous = step['counts']
                ends = [i for i, s in enumerate(steps) if s['counts'] in [w['counts'] for w in self.WAYPOINTS]]
                self.assertEqual([steps[i]['counts'] for i in ends], [w['counts'] for w in self.WAYPOINTS])
                self.assertEqual(steps[-1]['label'].split(' ')[0], 'B')

    def test_coordinated_steps_are_the_waypoints(self):
        steps = coordinated_steps(PARK_COUNTS, self.WAYPOINTS + self.WAYPOINTS[-1:])
        self.assertEqual([s['counts'] for s in steps], [w['counts'] for w in self.WAYPOINTS])

    def test_timing_follows_the_controller_smoothstep(self):
        steps = timed_steps(PARK_COUNTS, self.WAYPOINTS, {'mode': 'coordinated', 'speed': 20, 'step_counts': 8})
        self.assertAlmostEqual(steps[0]['seconds'], PEAK * 177 / 20)   # S3 580 -> 403 is the largest change
        self.assertEqual(position([0] * 6, [100] * 6, 0.5), [50.0] * 6)
        self.assertEqual(position([0] * 6, [100] * 6, 1), [100] * 6)
        self.assertLess(position([0] * 6, [100] * 6, 0.25)[0], 25)


class RehearsalTests(unittest.TestCase):
    def test_planned_tasks_pass_in_both_modes(self):
        for text in ('blue on orange', 'green to 5 15', 'grab orange', 'red on blue home', 'home'):
            for mode in ('single_servo', 'coordinated'):
                with self.subTest(text=text, mode=mode):
                    _, steps, result = run(text, params=with_overrides(motion={'mode': mode}))
                    self.assertTrue(result['verdict']['ok'], result['verdict']['text'])
                    times = [f['t'] for f in result['frames']]
                    self.assertEqual(times, sorted(times))
                    self.assertAlmostEqual(times[-1], sum(s['seconds'] for s in steps), places=2)
                    self.assertEqual(result['frames'][-1]['step'], len(steps))

    def test_frames_carry_the_cubes_only_when_one_moved(self):
        _, _, result = run('grab red')
        frames = result['frames']
        self.assertIn('cubes', frames[0])
        self.assertLess(sum('cubes' in f for f in frames), len(frames))
        held = frames[-1].get('cubes') or next(f['cubes'] for f in reversed(frames) if 'cubes' in f)
        self.assertTrue(next(c for c in held if c['id'] == 'red')['held'])
        self.assertEqual([e['kind'] for e in result['events']], ['grasp'])

    def test_contacts_that_begin_at_park_are_resting(self):
        _, _, result = run('home')
        kinds = {c['kind'] for c in result['contacts']}
        self.assertIn('resting', kinds)
        self.assertNotIn('contact', kinds)
        self.assertNotIn('away from PARK', result['verdict']['text'])

    def test_hard_contacts_fail_the_plan_only_when_asked(self):
        _, _, result = run('blue on orange')
        self.assertIn('contact', {c['kind'] for c in result['contacts']})
        self.assertTrue(result['verdict']['ok'])
        _, _, result = run('blue on orange', params=with_overrides(acceptance={'fail_on_contact': True}))
        self.assertFalse(result['verdict']['ok'])
        self.assertIn('away from PARK', result['verdict']['text'])

    def test_a_placement_outside_the_tolerance_fails(self):
        _, _, result = run('blue on orange', params=with_overrides(acceptance={'place_xy': 0.05}))
        self.assertFalse(result['verdict']['ok'])
        self.assertGreater(result['verdict']['offset_mm'], 0.5)


class TrajectoryPlannerTests(unittest.TestCase):
    def setUp(self):
        self.arm = SimulatedArm(PreviewDriver(), blocking=True, rest=PARK)
        self.planner = TrajectoryPlanner(self.arm)

    def test_a_plan_is_json_and_leaves_the_simulated_arm_alone(self):
        before = self.arm.snapshot()
        result = self.planner.plan(task='grab red', start=PARK_COUNTS)
        json.dumps(result)
        self.assertEqual(self.arm.snapshot(), before)
        self.assertEqual(result['description'], 'Pick red and hold it above its place')
        self.assertTrue(result['verdict']['ok'])
        self.assertEqual(result['steps'][-1]['counts'], result['waypoints'][-1]['counts'])
        self.assertEqual([c['id'] for c in result['scene']], ['orange', 'blue', 'red', 'green'])

    def test_requests_are_checked(self):
        for request, reason in (({'start': [700] + PARK_COUNTS[1:]}, 'start: S1 700 outside'),
                                ({'home': HOME_COUNTS[:5] + [10]}, 'home: S6 10 outside'),
                                ({'scene': [{'colour': 'purple', 'x': 0, 'y': 15}]}, 'colour'),
                                ({'parameters': {'motion': {'speed': 500}}}, 'motion.speed'),
                                ({'task': 'home', 'start': HOME_COUNTS, 'home': HOME_COUNTS}, 'already at the task goal')):
            with self.subTest(request=request), self.assertRaisesRegex(ValueError, reason):
                self.planner.plan(**{'task': 'grab red', 'start': PARK_COUNTS, **request})

    def test_planning_json_is_read_on_every_request(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'planning.json'
            edited = json.loads(json.dumps(DEFAULTS))
            edited['motion']['mode'] = 'coordinated'
            path.write_text(json.dumps(edited))
            with mock.patch.object(trajectory_planner, 'PLANNING_JSON', path):
                self.assertEqual(self.planner.defaults()['parameters']['motion']['mode'], 'coordinated')
                self.assertEqual(len(self.planner.plan(task='grab red', start=PARK_COUNTS)['steps']),
                                 len(self.planner.plan(task='grab red', start=PARK_COUNTS)['waypoints']))
                edited['approach']['hover'] = -1
                path.write_text(json.dumps(edited))
                with self.assertRaisesRegex(ValueError, 'planning.json: approach.hover'):
                    self.planner.defaults()


if __name__ == '__main__':
    unittest.main()
