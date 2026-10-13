"""The mock controller's trajectory planning (roboter_arm.sim.domain.planning), for the page's
"Trajectory planning" section.

A plan request is planned and rehearsed in its own copy of a cube scene. It never touches the
session, the driver or the simulated arm's world, and a --hardware controller does not have it.
planning.json is read on every request, so edits to it apply without a restart.
"""
import json

from roboter_arm.sim.domain.planning.parameters import PLANNING_JSON, resolve, rules, scene_entries
from roboter_arm.sim.domain.planning.planner import describe, guard_violations, plan_waypoints
from roboter_arm.sim.domain.planning.rehearsal import cube_states, rehearse
from roboter_arm.sim.domain.planning.steps import timed_steps
from roboter_arm.sim.domain.planning.task import parse_task
from roboter_arm.sim.domain.world import World


class TrajectoryPlanner:
    def __init__(self, arm):
        self.arm = arm      # the SimulatedArm: its PARK marks contacts there as resting

    def defaults(self):
        """planning.json, validated: {parameters, scene, rules}."""
        try:
            document = json.loads(PLANNING_JSON.read_text())
            parameters = resolve(document)
            World().set_scene(scene_entries(document.get('scene')))
        except ValueError as error:
            raise ValueError(f'planning.json: {error}') from None
        return {'parameters': parameters, 'scene': document['scene'], 'rules': rules()}

    def plan(self, *, task, start, home=None, scene=None, parameters=None):
        defaults = self.defaults()
        params = resolve(defaults['parameters'], parameters)
        for name, counts in (('start', start), ('home', home)):
            if counts is not None and (bad := guard_violations(counts)):
                raise ValueError(f'{name}: {", ".join(bad)}')
        world = World()
        world.set_scene(scene_entries(defaults['scene'] if scene is None else scene))
        initial = cube_states(world)
        parsed = parse_task(task, world)
        waypoints, expect = plan_waypoints(world, parsed, start, params, home=home)
        steps = timed_steps(start, waypoints, params['motion'])
        if not steps:
            raise ValueError('The arm is already at the task goal; nothing to do')
        rehearsal = rehearse(world, start, steps, expect, params, rest=list(self.arm.rest))
        for step in steps:
            step['seconds'] = round(step['seconds'], 3)
        return {'task': task, 'description': describe(parsed, expect), 'parameters': params, 'start': list(start),
                'home': home, 'scene': initial, 'expect': expect, 'waypoints': waypoints, 'steps': steps,
                'seconds': round(sum(step['seconds'] for step in steps), 3), **rehearsal}
