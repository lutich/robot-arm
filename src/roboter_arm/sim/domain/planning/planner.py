"""Pick-and-place waypoints in servo counts (from px-robotic-arm-sim planner.js planTransfer).

The gripper stays vertical: every arm waypoint is a top-down IK solution for the cube centre
(kinematics.ik_top_down), squared to the cube's yaw. The waypoints are

  Open gripper, Above <cube>, Descend 1..n, Grip <cube>, Lift 1..n            (pick)
  Above <target>, Lower 1..n, Release, Withdraw 1..n                          (place)
  HOME                                                                        (a task that says home)

The hover is the highest of approach.hover (or hover_stacked) and its fallbacks that both ends
reach. Every waypoint must lie inside the servo bounds of arm_model.json. No collision check:
the rehearsal reports contacts instead.
"""
import math

from roboter_arm.sim.domain.arm_model import CUBE, SERVOS
from roboter_arm.sim.domain.kinematics import Unreachable, angles_from_counts, counts_from_angles, ik_top_down
from roboter_arm.sim.domain.planning.task import PlanError
from roboter_arm.sim.domain.world import Cube, World


def guard_violations(counts, servos=SERVOS):
    """'S2 700 outside 185..660' for every count outside its servo's bounds."""
    return [f'{s.name} {c} outside {s.low}..{s.high}' for s, c in zip(servos, counts) if not s.low <= c <= s.high]


def cube_on(world, c):
    """The cube resting directly on top of cube c, if any."""
    return next((o for o in world.cubes if o is not c and not o.held and abs(o.p[2] - c.p[2] - CUBE) < 0.3
                 and World.in_footprint(c, o.p[0], o.p[1], 0.3)), None)


def top_of(world, c):
    """The cube at the top of the stack that contains c."""
    while (above := cube_on(world, c)) is not None:
        c = above
    return c


def plan_waypoints(world, task, start, params, *, home=None, servos=SERVOS):
    """Waypoints [{label, phase, counts}] from `start` (six counts) and what the plan should achieve:
    {type: 'pick' | 'place' | 'pose', cube, target (cube centre), on, hover}."""
    approach, s6, hold = params['approach'], servos[5], params['gripper']['hold']
    waypoints = []

    def push(label, phase, five, grip):
        counts = [*five, grip]
        if not waypoints or waypoints[-1]['counts'] != counts:
            waypoints.append({'label': label, 'phase': phase, 'counts': counts})

    def go_home(label, grip):
        if home is None:
            raise PlanError('the task ends at HOME, but no HOME pose was given')
        push(label, 'return', home[:5], grip)

    if task.pick is None:
        go_home('HOME', home[5] if home else None)
        return waypoints, {'type': 'pose', 'counts': list(home), 'hover': None}

    c = world.cube(task.pick)
    if c.held:
        raise PlanError(f'{c.id} is already in the gripper')
    if c.p[2] < 0:
        raise PlanError(f'{c.id} is off the board')
    if (above := cube_on(world, c)) is not None:
        raise PlanError(f'{c.id} has {above.id} on top; move {above.id} first')

    # Where the cube centre goes.
    tx = ty = tz = None
    tyaw, on = 0.0, None
    if task.on:
        base = world.cube(task.on)
        if base is c:
            raise PlanError('cannot put a cube on itself')
        top = top_of(world, base)
        if top is c:
            raise PlanError(f'{c.id} already sits on {base.id}')
        tx, ty, tz, tyaw, on = top.p[0], top.p[1], top.p[2] + CUBE, top.yaw, top.id
    elif task.at:
        tx, ty = task.at
        z, on, edge = world.support_for(Cube('_probe', '_probe', [tx, ty, 100.0]))
        if edge is not None:
            raise PlanError(f'({tx:g}, {ty:g}) is on the edge of {edge.id}; it would topple')
        if on == 'floor':
            raise PlanError(f'({tx:g}, {ty:g}) is off the board')
        tz = z + CUBE / 2

    s5_near = angles_from_counts(start, servos)[4]

    def solve(x, y, zc, yaw, label):
        nonlocal s5_near
        try:
            angles = ik_top_down(x, y, zc, cube_yaw=yaw, s5_near=s5_near, servos=servos)
        except Unreachable as error:
            raise PlanError(f'{label}: unreachable ({error})') from None
        s5_near = angles[4]
        counts = counts_from_angles(angles, s6.open, servos)
        if bad := guard_violations(counts, servos):
            raise PlanError(f'{label}: {", ".join(bad)}')
        return counts[:5]

    # The highest hover that both ends reach (stacks get the shorter one).
    gap = approach['release_gap']
    pick_hover = approach['hover'] if c.p[2] < CUBE else approach['hover_stacked']
    want = min(pick_hover, approach['hover'] if tz < CUBE else approach['hover_stacked']) if task.place else pick_hover
    hover = None
    for h in [want, *(h for h in approach['hover_fallbacks'] if h < want)]:
        try:
            solve(c.p[0], c.p[1], c.p[2] + h, c.yaw, 'probe')
            if task.place:
                solve(tx, ty, tz + gap + h, tyaw, 'probe')
        except PlanError:
            continue
        hover = h
        break
    s5_near = angles_from_counts(start, servos)[4]
    if hover is None:                       # say why, at the lowest hover
        hover = min([want, *approach['hover_fallbacks']])
        solve(c.p[0], c.p[1], c.p[2] + hover, c.yaw, f'above {c.id}')
        if task.place:
            solve(tx, ty, tz + gap + hover, tyaw, f'above {on}')

    def vertical(x, y, z0, z1, yaw, grip, label, phase):
        n = max(1, math.ceil(abs(z1 - z0) / approach['descent_step'] - 1e-9))
        for i in range(1, n + 1):
            push(f'{label} {i}/{n}', phase, solve(x, y, z0 + (z1 - z0) * i / n, yaw, label), grip)

    push('Open gripper', 'pick', start[:5], s6.open)
    push(f'Above {c.id}', 'pick', solve(c.p[0], c.p[1], c.p[2] + hover, c.yaw, f'above {c.id}'), s6.open)
    vertical(c.p[0], c.p[1], c.p[2] + hover, c.p[2], c.yaw, s6.open, 'Descend', 'pick')
    push(f'Grip {c.id}', 'pick', waypoints[-1]['counts'][:5], hold)
    vertical(c.p[0], c.p[1], c.p[2], c.p[2] + hover, c.yaw, hold, 'Lift', 'carry')
    if not task.place:
        expect = {'type': 'pick', 'cube': c.id, 'target': list(c.p), 'on': None, 'hover': hover}
        if task.home:
            go_home(f'HOME holding {c.id}', hold)
        return waypoints, expect

    z = tz + gap
    push(f'Above {on}' if on != 'board' else f'Above ({tx:g}, {ty:g})', 'carry', solve(tx, ty, z + hover, tyaw, 'above target'), hold)
    vertical(tx, ty, z + hover, z, tyaw, hold, 'Lower', 'place')
    push('Release', 'place', waypoints[-1]['counts'][:5], s6.open)
    vertical(tx, ty, z, z + hover, tyaw, s6.open, 'Withdraw', 'place')
    if task.home:
        go_home('HOME', home[5] if home else None)
    return waypoints, {'type': 'place', 'cube': c.id, 'target': [tx, ty, tz], 'on': on, 'hover': hover}


def describe(task, expect):
    """One line for the panel: what the plan does."""
    if expect['type'] == 'pose':
        return 'Go to HOME'
    if expect['type'] == 'pick':
        what = f'Pick {expect["cube"]} and hold it' + ('' if task.home else ' above its place')
    elif expect['on'] == 'board':
        what = f'Pick {expect["cube"]} and place it at ({expect["target"][0]:.1f}, {expect["target"][1]:.1f}) cm'
    else:
        what = f'Pick {expect["cube"]} and place it on {expect["on"]}'
    return what + (', then return to HOME' if task.home else '')
