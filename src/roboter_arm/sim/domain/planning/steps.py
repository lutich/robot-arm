"""Waypoints -> the steps the controller executes, and how long each takes.

single_servo (px-robotic-arm-sim core/planning.js controllerCompatiblePath): each waypoint-to-waypoint
move is cut into equal parts of at most step_counts on its largest joint, and each part is written
one servo at a time, S1 to S6. Every step changes exactly one servo, which is what POST /api/move does.
coordinated: one step per waypoint, all servos together, which is what demo playback does.

Timing follows the controller (control/domain/motion.py): a step whose largest change is d counts
takes PEAK * d / speed seconds on a cubic smoothstep.
"""
import math

PEAK = 1.5   # a cubic smoothstep peaks at 1.5 times its average speed


def single_servo_steps(start, waypoints, step_counts):
    steps, cursor = [], list(start)
    for waypoint in waypoints:
        origin, target = list(cursor), waypoint['counts']
        parts = max(1, math.ceil(max(abs(b - a) for a, b in zip(origin, target)) / step_counts))
        segment = []
        for k in range(1, parts + 1):
            sample = [round(a + (b - a) * k / parts) for a, b in zip(origin, target)]
            for channel in range(6):
                if cursor[channel] != sample[channel]:
                    cursor = list(cursor)
                    cursor[channel] = sample[channel]
                    segment.append(cursor)
        for i, counts in enumerate(segment, 1):
            label = f'{waypoint["label"]} ({i}/{len(segment)})' if len(segment) > 1 else waypoint['label']
            steps.append({'label': label, 'phase': waypoint['phase'], 'counts': counts})
    return steps


def coordinated_steps(start, waypoints):
    steps, cursor = [], list(start)
    for waypoint in waypoints:
        if waypoint['counts'] != cursor:
            steps.append({'label': waypoint['label'], 'phase': waypoint['phase'], 'counts': list(waypoint['counts'])})
            cursor = waypoint['counts']
    return steps


def step_seconds(origin, target, speed):
    return PEAK * max(abs(b - a) for a, b in zip(origin, target)) / speed


def timed_steps(start, waypoints, motion):
    """The steps of motion['mode'], each with its duration in seconds."""
    steps = (single_servo_steps(start, waypoints, motion['step_counts']) if motion['mode'] == 'single_servo'
             else coordinated_steps(start, waypoints))
    previous = start
    for step in steps:
        step['seconds'] = step_seconds(previous, step['counts'], motion['speed'])
        previous = step['counts']
    return steps


def position(origin, target, fraction):
    """Counts (not rounded) `fraction` of the way through a step, on the controller's smoothstep."""
    p = fraction * fraction * (3 - 2 * fraction)
    return [a + (b - a) * p for a, b in zip(origin, target)]
