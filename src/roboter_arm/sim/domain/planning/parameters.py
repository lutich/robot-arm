"""Trajectory planning parameters. planning.json next to this file holds the defaults, the only copy.

Edit planning.json to change them for everyone: the mock controller reads it on every request, so
no restart is needed. A plan request may also carry overrides to try values without saving them.
Lengths in cm (board frame, as arm_model.json), counts in PCA9685 counts at 60 Hz, times in seconds.

motion      mode           single_servo: every step moves one servo, as POST /api/move does: each
                           waypoint-to-waypoint move is cut into parts of at most step_counts on its
                           largest joint, written S1..S6. coordinated: all servos move together from
                           waypoint to waypoint, as demo playback does.
            speed          counts/s. A step whose largest change is d counts takes 1.5 d / speed on the
                           controller's cubic smoothstep (control/domain/motion.py). The controller runs
                           single-joint moves at 20 and demo playback at its demo speed (1..100).
            step_counts    largest change of one single_servo step.
approach    hover          height of the cube centre above its pick or place position where the
                           gripper arrives before going straight down.
            hover_stacked  the same for a cube picked from or placed on a stack (level 1 and up).
            hover_fallbacks lower hovers, tried in order when the arm cannot reach the hover.
            descent_step   vertical moves are a top-down IK waypoint every descent_step, so the gripper
                           stays vertical on the way down and up.
            release_gap    the jaws open this far above the support; the cube drops onto it.
gripper     hold           S6 count commanded to hold a cube. The jaws touch a 2,65 cm cube at about
                           S6 = 567 (kinematics.counts_for_gap); the counts beyond squeeze it.
                           Opening uses S6 open from arm_model.json.
acceptance  place_xy       a placed cube passes within this distance of its target centre ...
            place_z        ... and this height difference.
            fail_on_contact a plan fails if the rehearsal shows a hard contact (arm into the board,
                           itself or a cube) away from PARK. Off by default: the model's servo
                           calibration is not measured, so contacts are only reported.
rehearsal   frame_seconds  the rehearsal steps the cube world and records a frame this often.
scene       the cubes at the start: colour (orange, blue, yellow, green or red), x, y on the board,
            level (0 on the board, 1 on top of another cube ...), yaw in degrees.
"""
import math
from pathlib import Path

from roboter_arm.sim.domain.arm_model import SERVOS

PLANNING_JSON = Path(__file__).with_name('planning.json')
MAX_CUBES = 12
S6 = SERVOS[5]

# group -> key -> ('number' | 'integer', low, high) | ('numbers', low, high) | ('choice', options...) | ('flag',)
RULES = {
    'motion': {'mode': ('choice', 'single_servo', 'coordinated'), 'speed': ('number', 1, 100),
               'step_counts': ('integer', 1, 200)},
    'approach': {'hover': ('number', 0.5, 15), 'hover_stacked': ('number', 0.5, 15),
                 'hover_fallbacks': ('numbers', 0.5, 15), 'descent_step': ('number', 0.1, 5),
                 'release_gap': ('number', 0, 2)},
    'gripper': {'hold': ('integer', S6.open, S6.closed)},
    'acceptance': {'place_xy': ('number', 0.05, 5), 'place_z': ('number', 0.05, 2), 'fail_on_contact': ('flag',)},
    'rehearsal': {'frame_seconds': ('number', 0.005, 0.5)},
}


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def check(path, value, rule):
    kind = rule[0]
    if kind == 'choice':
        ok, want = value in rule[1:], ' or '.join(rule[1:])
    elif kind == 'flag':
        ok, want = type(value) is bool, 'true or false'
    elif kind == 'numbers':
        ok = type(value) is list and len(value) <= 8 and all(finite(v) and rule[1] <= v <= rule[2] for v in value)
        want = f'a list of up to 8 numbers from {rule[1]} to {rule[2]}'
    else:
        ok = (type(value) is int if kind == 'integer' else finite(value)) and rule[1] <= value <= rule[2]
        want = f'{"an integer" if kind == "integer" else "a number"} from {rule[1]} to {rule[2]}'
    if not ok:
        raise ValueError(f'{path} must be {want}')


def resolve(defaults, overrides=None):
    """The parameter groups of `defaults` (planning.json) with `overrides` applied, validated.
    Overrides name only groups and keys that exist; a missing default is an error too."""
    if overrides is not None and not isinstance(overrides, dict):
        raise ValueError('parameters must be an object of groups')
    for group, values in (overrides or {}).items():
        if group not in RULES or not isinstance(values, dict):
            raise ValueError(f'unknown parameter group {group}')
        for key in values:
            if key not in RULES[group]:
                raise ValueError(f'unknown parameter {group}.{key}')
    params = {}
    for group, rules in RULES.items():
        params[group] = {}
        for key, rule in rules.items():
            given = (overrides or {}).get(group, {})
            if key in given:
                value = given[key]
            elif key in defaults.get(group, {}):
                value = defaults[group][key]
            else:
                raise ValueError(f'{group}.{key} is missing')
            check(f'{group}.{key}', value, rule)
            params[group][key] = list(value) if isinstance(value, list) else value
    return params


def rules():
    """The validation rules for clients (the panel's parameter form)."""
    described = {}
    for group, keys in RULES.items():
        for key, rule in keys.items():
            kind = rule[0]
            entry = {'kind': kind}
            if kind == 'choice':
                entry['choices'] = list(rule[1:])
            elif kind != 'flag':
                entry['min'], entry['max'] = rule[1], rule[2]
            described[f'{group}.{key}'] = entry
    return described


def scene_entries(scene):
    """(colour, x, y, level, yaw) tuples for World.set_scene, which checks colours and the board."""
    if not isinstance(scene, list) or len(scene) > MAX_CUBES:
        raise ValueError(f'scene must be a list of at most {MAX_CUBES} cubes')
    entries = []
    for index, cube in enumerate(scene, 1):
        if not isinstance(cube, dict) or not {'colour', 'x', 'y'} <= set(cube) <= {'colour', 'x', 'y', 'level', 'yaw'}:
            raise ValueError(f'scene cube {index} needs colour, x and y, optionally level and yaw')
        level, yaw = cube.get('level', 0), cube.get('yaw', 0)
        if not (isinstance(cube['colour'], str) and finite(cube['x']) and finite(cube['y'])
                and type(level) is int and finite(yaw)):
            raise ValueError(f'scene cube {index}: colour is a name, x, y and yaw are numbers, level an integer')
        entries.append((cube['colour'], cube['x'], cube['y'], level, yaw))
    return entries
