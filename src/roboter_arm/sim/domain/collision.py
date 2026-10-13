"""Collision primitives and the arm's collision model (from threejs-sim collision.js).

Every query returns a signed separation in cm: > 0 apart, < 0 penetrating. Box-box uses the
separating axis test, so its positive value is a lower bound of the distance; enough for
"collides or not" with a small tolerance.
"""
from dataclasses import dataclass, field
import math

from roboter_arm.sim.domain.arm_model import ARM, CLEARANCE, CUBE, GRIPPER, PRESS_DEPTH
from roboter_arm.sim.domain.kinematics import (
    WORLD, apply3, pad_depth_for_theta, point, rot_z, theta_for_gap, transpose3)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


@dataclass
class Capsule:
    a: tuple
    b: tuple
    r: float
    name: str = ''
    link: int = 0
    kind: str = field(default='capsule', init=False)

    def bound(self):
        """Bounding sphere (centre, radius) for the broad phase."""
        return tuple((self.a[i] + self.b[i]) / 2 for i in range(3)), math.dist(self.a, self.b) / 2 + self.r

    def lowest_z(self):
        return min(self.a[2], self.b[2]) - self.r

    def json(self):
        return {'kind': 'capsule', 'name': self.name, 'a': self.a, 'b': self.b, 'r': self.r}


@dataclass
class Box:
    c: tuple
    R: tuple       # columns = box axes in world
    h: tuple       # half sizes
    name: str = ''
    link: int = 0
    held: bool = False
    kind: str = field(default='box', init=False)

    def bound(self):
        return self.c, math.sqrt(dot(self.h, self.h))

    def lowest_z(self):
        R = self.R
        return self.c[2] - (abs(R[6]) * self.h[0] + abs(R[7]) * self.h[1] + abs(R[8]) * self.h[2])

    def json(self):
        return {'kind': 'box', 'name': self.name, 'c': self.c, 'R': self.R, 'h': self.h}


# ---------------- distances ----------------

def seg_seg_dist(p1, q1, p2, q2):
    """Closest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = sub(q1, p1), sub(q2, p2), sub(p1, p2)
    a, e, f = dot(d1, d1), dot(d2, d2), dot(d2, r)
    clamp = lambda v: min(1.0, max(0.0, v))
    if a <= 1e-12 and e <= 1e-12:
        return math.sqrt(dot(r, r))
    if a <= 1e-12:
        s, t = 0.0, clamp(f / e)
    else:
        c = dot(d1, r)
        if e <= 1e-12:
            t, s = 0.0, clamp(-c / a)
        else:
            b = dot(d1, d2)
            den = a * e - b * b
            s = clamp((b * f - c * e) / den) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, clamp(-c / a)
            elif t > 1:
                t, s = 1.0, clamp((b - c) / a)
    c1 = tuple(p1[i] + d1[i] * s for i in range(3))
    c2 = tuple(p2[i] + d2[i] * t for i in range(3))
    return math.dist(c1, c2)


def _local_point_box_dist(q, h):
    """Distance from a box-local point to the box; negative inside (depth to the nearest face)."""
    d = [abs(q[i]) - h[i] for i in range(3)]
    outside = math.sqrt(sum(max(v, 0) ** 2 for v in d))
    return outside if outside > 0 else max(d)


def seg_box_dist(a, b, box):
    """Signed distance between segment a-b and a box. Convex along the segment, so a
    golden-section search is exact enough."""
    RT = transpose3(box.R)
    pa, pb = apply3(RT, sub(a, box.c)), apply3(RT, sub(b, box.c))
    d = sub(pb, pa)
    f = lambda t: _local_point_box_dist((pa[0] + d[0] * t, pa[1] + d[1] * t, pa[2] + d[2] * t), box.h)
    lo, hi, g = 0.0, 1.0, 0.6180339887
    x1, x2 = hi - g * (hi - lo), lo + g * (hi - lo)
    f1, f2 = f(x1), f(x2)
    for _ in range(28):
        if f1 < f2:
            hi, x2, f2 = x2, x1, f1
            x1 = hi - g * (hi - lo)
            f1 = f(x1)
        else:
            lo, x1, f1 = x1, x2, f2
            x2 = lo + g * (hi - lo)
            f2 = f(x2)
    return min(f1, f2, f(0), f(1))


def _box_axes(box):
    return [(box.R[i], box.R[3 + i], box.R[6 + i]) for i in range(3)]


def box_box_sep(A, B):
    """Separating axis test: a positive separation, or minus the smallest overlap."""
    ax, bx, t = _box_axes(A), _box_axes(B), sub(B.c, A.c)
    best = -math.inf
    axes = ax + bx + [cross(u, v) for u in ax for v in bx]
    for L in axes:
        n = math.sqrt(dot(L, L))
        if n < 1e-6:
            continue
        ra = sum(A.h[i] * abs(dot(ax[i], L)) for i in range(3))
        rb = sum(B.h[i] * abs(dot(bx[i], L)) for i in range(3))
        sep = (abs(dot(t, L)) - ra - rb) / n
        best = max(best, sep)
        if sep > 0:
            return best
    return best


def separation(s1, s2):
    if s1.kind == 'capsule' and s2.kind == 'capsule':
        return seg_seg_dist(s1.a, s1.b, s2.a, s2.b) - s1.r - s2.r
    if s1.kind == 'capsule':
        return seg_box_dist(s1.a, s1.b, s2) - s1.r
    if s2.kind == 'capsule':
        return seg_box_dist(s2.a, s2.b, s1) - s2.r
    return box_box_sep(s1, s2)


# ---------------- shape builders ----------------

def box_in_frame(f, offset, h, name, link):
    return Box(point(f, offset), f.R, tuple(h), name, link)


def capsule_in_frame(f, a, b, r, name, link):
    return Capsule(point(f, a), point(f, b), r, name, link)


def cube_shape(cube):
    """A cube as a box: needs .p (centre) and .R or .yaw (degrees, upright cube)."""
    R = cube.R if cube.R is not None else rot_z(cube.yaw * math.pi / 180)
    return Box(tuple(cube.p), R, (CUBE / 2,) * 3, f'{cube.id} cube', held=cube.held)


# Neighbouring links always touch at their joint: pairs whose link indices differ by less
# than 2 are not tested against each other.
LINK = {'base': 0, 'turret': 1, 'upper': 2, 'fore': 3, 'wrist': 4, 'hand': 5, 'finger': 6}


def arm_shapes(f, gap):
    """Collision shapes for joint frames `f` (from fk) and the actual jaw gap.
    Sizes are from the photos and the MG996R body (4,07 x 1,97 x 4,29 cm), not measured."""
    L = LINK
    s = [
        # Base: two U beams, the S1 servo and its bracket; does not turn with S1.
        box_in_frame(WORLD, (0, 2.2, 1.25), (4.5, 4.2, 1.25), 'base', L['base']),
        box_in_frame(WORLD, (-1.0, 0, 4.5), (3.1, 1.4, 2.0), 'S1 servo', L['base']),
        # Turret: bracket on the S1 horn with the S2 servo lying toward the back.
        box_in_frame(f.base, (0, -0.8, 8.95), (2.7, 2.6, 1.95), 'turret', L['turret']),
        # Upper arm: side plates around the S3 servo, shortened at the joints.
        box_in_frame(f.shoulder, (0, 0, ARM['L2'] / 2), (2.9, 1.3, ARM['L2'] / 2 - 1.3), 'upper arm', L['upper']),
        # PCA9685 on its foam pad on the -X side plate, connectors included.
        box_in_frame(f.shoulder, (-4.35, -0.2, 4.5), (1.65, 1.7, 3.6), 'PCA9685', L['upper']),
        # Forearm: S3 servo behind the elbow, aluminium tube, S4 servo in its bracket.
        box_in_frame(f.elbow, (0, 0, -0.95), (2.1, 1.25, 2.55), 'S3 servo', L['fore']),
        capsule_in_frame(f.elbow, (0, 0, 1.9), (0, 0, ARM['L3'] - 2.8), 0.95, 'forearm', L['fore']),
        box_in_frame(f.elbow, (0, -1.0, ARM['L3'] - 0.9), (2.6, 2.1, 1.6), 'S4 bracket', L['fore']),
        # Wrist: U bracket from the S4 horn and the S5 servo body.
        box_in_frame(f.wrist, (0, 0, 1.4), (2.9, 1.3, 1.3), 'wrist bracket', L['wrist']),
        box_in_frame(f.wrist, (1.035, 0, ARM['L4'] - 2.335), (2.04, 0.99, 1.84), 'S5 servo', L['wrist']),
        # Gripper base plate with the gears, and the S6 servo behind it.
        box_in_frame(f.hand, (0, 0, 1.25), (GRIPPER['plate']['w'] / 2, 0.65, 0.95), 'gripper plate', L['hand']),
        box_in_frame(f.hand, (GRIPPER['gear'][0] + 1.035, -2.635, GRIPPER['gear'][1]), (2.04, 1.84, 0.99), 'S6 servo', L['hand']),
    ]
    # Fingers: link from the gear pivot, finger body down to the pad, and the pad itself.
    th = theta_for_gap(gap) * math.pi / 180
    v_pad = pad_depth_for_theta(theta_for_gap(gap))
    gear, link, pad = GRIPPER['gear'], GRIPPER['link'], GRIPPER['pad']
    for side, name in ((-1, 'left'), (1, 'right')):
        gu = side * gear[0]
        ge = (gu + side * link * math.sin(th), 0, gear[1] + link * math.cos(th))
        s.append(capsule_in_frame(f.hand, (gu, 0, gear[1]), ge, 0.35, f'{name} link', L['finger']))
        u_in = side * (gap / 2 + pad['t'])
        s.append(box_in_frame(f.hand, (u_in + side * 0.3, 0, (ge[2] + v_pad + 0.7) / 2),
                              (0.3, 0.4, (v_pad + 0.7 - ge[2]) / 2), f'{name} finger', L['finger']))
        s.append(box_in_frame(f.hand, (side * (gap / 2 + pad['t'] / 2), 0, v_pad),
                              (pad['t'] / 2, pad['w'] / 2, pad['h'] / 2), f'{name} pad', L['finger']))
    return s


# ---------------- scene queries ----------------

@dataclass(frozen=True)
class Contact:
    a: str
    b: str
    sep: float
    # Pressing: the fingers on the board, or a held cube on the board or a stack. The board
    # stops a real servo short of its command there, so the commanded pose overlaps it.
    pressing: bool = False


def _near(x, y):
    (cx, rx), (cy, ry) = x.bound(), y.bound()
    return math.dist(cx, cy) <= rx + ry


def on_board(board, x, y):
    return abs(x) <= board['w'] / 2 and -board['axisToBackEdge'] <= y <= board['d'] - board['axisToBackEdge']


def find_contacts(arm, cubes, board, tol=CLEARANCE):
    """Every pair penetrating by more than tol: arm self, arm-board, arm-cube, cube-cube, cube-board.
    A held cube moves with the hand, so it is not tested against the gripper."""
    out = []
    for i, A in enumerate(arm):
        for B in arm[i + 1:]:
            if abs(A.link - B.link) < 2 or not _near(A, B):
                continue
            sep = separation(A, B)
            if sep < -tol:
                out.append(Contact(A.name, B.name, sep))
    for A in arm:
        if A.link == LINK['base']:
            continue
        z = A.lowest_z()
        if z < -tol and on_board(board, *A.bound()[0][:2]):
            out.append(Contact(A.name, 'board', z, A.link == LINK['finger'] and z > -PRESS_DEPTH))
    shapes = [cube_shape(c) for c in cubes]
    for C in shapes:
        for A in arm:
            if (C.held and A.link >= LINK['hand']) or not _near(A, C):
                continue
            sep = separation(A, C)
            if sep < -tol:
                out.append(Contact(A.name, C.name, sep))
    for i, C in enumerate(shapes):
        z = C.lowest_z()
        if z < -tol and on_board(board, C.c[0], C.c[1]):
            out.append(Contact(C.name, 'board', z, C.held and z > -PRESS_DEPTH))
        for D in shapes[i + 1:]:
            if not _near(C, D):
                continue
            sep = separation(C, D)
            if sep < -tol:
                out.append(Contact(C.name, D.name, sep, (C.held or D.held) and sep > -PRESS_DEPTH))
    return out
