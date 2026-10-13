"""Forward and inverse kinematics, counts <-> angles, gripper linkage (from threejs-sim kinematics.js).

Model angles: 0 = all links straight up; positive S1 turns +Y toward +X; positive S2, S3, S4
tilt the arm toward +Y; S5 turns around the link axis. Rotations are row-major 3x3 tuples.
"""
from dataclasses import dataclass
import math

from roboter_arm.sim.domain.arm_model import ARM, CUBE, GRIPPER, SERVOS

RAD = math.pi / 180
DEG = 180 / math.pi

# ---------------- small rigid-transform helpers ----------------

I3 = (1, 0, 0, 0, 1, 0, 0, 0, 1)


def mul3(a, b):
    return tuple(sum(a[3 * i + k] * b[3 * k + j] for k in range(3)) for i in range(3) for j in range(3))


def apply3(R, v):
    return (R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
            R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
            R[6] * v[0] + R[7] * v[1] + R[8] * v[2])


def transpose3(R):
    return (R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return (1, 0, 0, 0, c, -s, 0, s, c)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return (c, -s, 0, s, c, 0, 0, 0, 1)


@dataclass(frozen=True)
class Frame:
    R: tuple
    p: tuple


WORLD = Frame(I3, (0, 0, 0))


def compose(f, R, offset=(0, 0, 0)):
    """Child frame expressed in the parent f."""
    o = apply3(f.R, offset)
    return Frame(mul3(f.R, R), (f.p[0] + o[0], f.p[1] + o[1], f.p[2] + o[2]))


def point(f, v):
    o = apply3(f.R, v)
    return (f.p[0] + o[0], f.p[1] + o[1], f.p[2] + o[2])


def axis(f, i):
    return (f.R[i], f.R[3 + i], f.R[6 + i])


# ---------------- counts <-> model angles ----------------

def angle_from_counts(servo, counts):
    return servo.q_ref + servo.direction * (counts - servo.ref) / servo.cpd


def counts_from_angle(servo, deg):
    return servo.ref + servo.direction * (deg - servo.q_ref) * servo.cpd


def angles_from_counts(counts, servos=SERVOS):
    """Model angles (s1..s5) in degrees from counts [c0..c5]."""
    return tuple(angle_from_counts(servos[i], counts[i]) for i in range(5))


def counts_from_angles(angles, s6, servos=SERVOS):
    """Rounded counts for S1..S5 from model angles; S6 is passed through."""
    return [round(counts_from_angle(servos[i], q)) for i, q in enumerate(angles)] + [s6]


# ---------------- gripper linkage ----------------

def gripper_theta(counts, s6=SERVOS[5]):
    """Link angle from vertical, degrees. Linear in counts between the measured end points."""
    t = (counts - s6.open) / (s6.closed - s6.open)
    return GRIPPER['thetaOpen'] + t * (GRIPPER['thetaClosed'] - GRIPPER['thetaOpen'])


def gap_for_theta(theta):
    return max(0.0, 2 * (GRIPPER['gear'][0] + GRIPPER['link'] * math.sin(theta * RAD) + GRIPPER['tip'][0]))


def pad_depth_for_theta(theta):
    return GRIPPER['gear'][1] + GRIPPER['link'] * math.cos(theta * RAD) + GRIPPER['tip'][1]


def theta_for_gap(gap):
    s = (gap / 2 - GRIPPER['gear'][0] - GRIPPER['tip'][0]) / GRIPPER['link']
    return math.asin(max(-1.0, min(1.0, s))) * DEG


def counts_for_gap(width, s6=SERVOS[5]):
    """S6 counts at which the pads first touch a face of this width."""
    th = theta_for_gap(width)
    return s6.open + (th - GRIPPER['thetaOpen']) / (GRIPPER['thetaClosed'] - GRIPPER['thetaOpen']) * (s6.closed - s6.open)


# ---------------- forward kinematics ----------------

@dataclass(frozen=True)
class Frames:
    """Frames of every joint. The hand frame sits at the flange: +Z is the tool axis (away from
    the wrist), +X the jaw closing axis, +Y the jaw depth axis."""
    base: Frame
    shoulder: Frame
    elbow: Frame
    wrist: Frame
    hand: Frame
    grasp: tuple


def fk(angles):
    s1, s2, s3, s4, s5 = angles
    base = compose(WORLD, rot_z(-s1 * RAD))
    shoulder = compose(base, rot_x(-s2 * RAD), (0, 0, ARM['H_S2']))
    elbow = compose(shoulder, rot_x(-s3 * RAD), (0, 0, ARM['L2']))
    wrist = compose(elbow, rot_x(-s4 * RAD), (0, 0, ARM['L3']))
    hand = compose(wrist, rot_z(s5 * RAD), (0, 0, ARM['L4']))
    return Frames(base, shoulder, elbow, wrist, hand, point(hand, (0, 0, ARM['G_BLOCK'])))


def tool_yaw_and_tilt(hand):
    """World yaw (deg, CCW from +X) of the jaw closing axis, and how far the tool is from vertical."""
    x, z = axis(hand, 0), axis(hand, 2)
    return math.atan2(x[1], x[0]) * DEG, math.acos(max(-1.0, min(1.0, -z[2]))) * DEG


# ---------------- inverse kinematics (top-down grasp) ----------------

class Unreachable(ValueError):
    pass


def wrap90(d):
    return ((d + 45) % 90 + 90) % 90 - 45


def ik_top_down(x, y, zc, *, cube_yaw=0.0, s5_near=0.0, servos=SERVOS):
    """Model angles with the held cube centre at (x, y, zc), gripper straight down, jaws square
    to a cube of world yaw `cube_yaw`. `s5_near` picks among the S5 solutions 90 degrees apart."""
    tool = ARM['L4'] + ARM['G_BLOCK']
    theta1 = math.atan2(x, y) * DEG
    r = math.hypot(x, y)
    dz = zc + tool - ARM['H_S2']
    d = math.hypot(r, dz)
    L2, L3 = ARM['L2'], ARM['L3']
    if not abs(L2 - L3) <= d <= L2 + L3:
        raise Unreachable(f'wrist point {d:.2f} cm from S2, reach is {abs(L2 - L3):.1f} to {L2 + L3:.1f}')
    c3 = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    q3 = -math.acos(max(-1.0, min(1.0, c3)))                    # elbow up
    phi2 = math.atan2(dz, r) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    q4 = -math.pi / 2 - (phi2 + q3)
    # Jaw yaw with the tool down is -s1 - s5; square to the cube every 90 degrees.
    base = wrap90(-theta1 - cube_yaw)
    s5_servo = servos[4]
    ends = angle_from_counts(s5_servo, s5_servo.low), angle_from_counts(s5_servo, s5_servo.high)
    choices = [base + 90 * k for k in range(-3, 4) if min(ends) <= base + 90 * k <= max(ends)]
    if not choices:
        raise Unreachable('no wrist rotation inside the S5 guards')
    s5 = min(choices, key=lambda c: abs(c - s5_near))
    return (theta1, 90 - phi2 * DEG, -q3 * DEG, -q4 * DEG, s5)


def max_reach(zc=CUBE / 2):
    """Largest horizontal distance from S1 for a top-down grasp with the cube centre at zc."""
    dz = zc + ARM['L4'] + ARM['G_BLOCK'] - ARM['H_S2']
    return math.sqrt((ARM['L2'] + ARM['L3']) ** 2 - dz * dz)
