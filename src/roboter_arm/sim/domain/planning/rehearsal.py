"""Run planned steps in a copy of the cube world and judge the outcome (px-robotic-arm-sim verify()).

The world is world.py: grasping by rule, no rigid-body physics, so cubes stay upright and a
released cube drops onto its support. Every step is sampled on the controller's smoothstep each
rehearsal.frame_seconds. A frame holds the time, the step number (0 = start), the counts and the
jaw gap, and the cubes whenever one of them moved.

Contacts (collision.py) are reported once per pair and kind, with the deepest penetration:
pressing (fingers or a held cube up to press_depth into the board), resting (begun within
rest_zone counts of PARK, where the gripper lies on the base, and not interrupted since) or
contact (anything else). Like the mock controller's SimulatedArm, a contact keeps the kind it
began with until it has been clear for more than FLICKER_FRAMES frames.
"""
import math

from roboter_arm.sim.domain.arm_model import REST_ZONE
from roboter_arm.sim.domain.planning.steps import position

FLICKER_FRAMES = 3


def rounded(value, digits):
    if isinstance(value, (list, tuple)):
        return [rounded(v, digits) for v in value]
    return round(value, digits) if isinstance(value, float) else value


def cube_states(world):
    return [{**c.json(), 'p': rounded(c.p, 3), 'yaw': round(c.yaw, 2), 'R': rounded(c.R, 4) if c.R else None}
            for c in world.cubes]


def rehearse(world, start, steps, expect, params, *, rest):
    """Run `steps` from `start` in `world` (changed in place). `rest`: PARK counts.
    Returns {frames, events, contacts, verdict}."""
    dt = params['rehearsal']['frame_seconds']
    frames, events, contacts, seen, active = [], [], [], {}, {}
    clock = {'t': 0.0, 'seq': world.seq, 'cubes': None}

    def record(index, counts):
        found = world.step(counts)
        resting = all(abs(c - r) <= REST_ZONE for c, r in zip(counts, rest))
        for k in found:
            began = active.get((k.a, k.b))
            if k.pressing:
                kind = 'pressing'
            elif began and len(frames) - began[1] <= FLICKER_FRAMES and began[0] != 'pressing':
                kind = began[0]
            else:
                kind = 'resting' if resting else 'contact'
            active[k.a, k.b] = (kind, len(frames))
            depth = round(max(0.0, -k.sep) * 10, 1)
            entry = seen.get((k.a, k.b, kind))
            if entry is None:
                seen[k.a, k.b, kind] = entry = {'a': k.a, 'b': k.b, 'kind': kind, 't': round(clock['t'], 3), 'depth_mm': depth}
                contacts.append(entry)
            entry['depth_mm'] = max(entry['depth_mm'], depth)
        for event in world.events:
            if event['seq'] > clock['seq']:
                events.append({'t': round(clock['t'], 3), 'kind': event['kind'], 'text': event['text']})
        clock['seq'] = world.seq
        frame = {'t': round(clock['t'], 3), 'step': index, 'counts': rounded(list(counts), 2), 'gap': round(world.gap, 3)}
        cubes = cube_states(world)
        if cubes != clock['cubes']:
            frame['cubes'] = clock['cubes'] = cubes
        frames.append(frame)

    record(0, [float(c) for c in start])
    counts = start
    for index, step in enumerate(steps, 1):
        n = max(1, math.ceil(step['seconds'] / dt - 1e-9))
        for k in range(1, n + 1):
            clock['t'] += step['seconds'] / n
            record(index, position(counts, step['counts'], k / n))
        counts = step['counts']
    hard = [k for k in contacts if k['kind'] == 'contact']
    return {'frames': frames, 'events': events, 'contacts': contacts,
            'verdict': judge(world, expect, counts, params['acceptance'], hard)}


def judge(world, expect, counts, acceptance, hard):
    if expect['type'] == 'pose':
        ok, text, offset = list(counts) == expect['counts'], 'reached HOME', None
    else:
        cube = world.cube(expect['cube'])
        if expect['type'] == 'pick':
            ok = world.holding is not None and world.holding.id == cube.id
            text, offset = f'{cube.id} is held' if ok else f'{cube.id} was not held', None
        else:
            x, y, z = expect['target']
            offset = math.hypot(cube.p[0] - x, cube.p[1] - y)
            ok = not cube.held and offset < acceptance['place_xy'] and abs(cube.p[2] - z) < acceptance['place_z']
            where = f'{offset * 10:.1f} mm from its target'
            text = f'{cube.id} placed {where}' if ok else f'{cube.id} ended {where}' + (', still held' if cube.held else '')
    if hard:
        text += f'; {len(hard)} contact{"s" if len(hard) > 1 else ""} away from PARK'
        if acceptance['fail_on_contact']:
            ok = False
    return {'ok': ok, 'text': text, 'offset_mm': None if offset is None else round(offset * 10, 1)}
