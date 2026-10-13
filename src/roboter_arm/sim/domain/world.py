"""The simulated world: arm pose from servo counts, rule-based grasping, cubes, contacts.

No rigid-body physics (threejs-sim physics.js is not ported). Cubes stand upright on the board
or on each other. Grasping follows threejs-sim world.js: when the closing jaws reach a cube that
sits between the pads, it is squared up and moves rigidly with the hand. Opening the jaws drops
it onto the highest support below its centre; cubes left without support drop too.
"""
from dataclasses import dataclass, field
import copy
import math

from roboter_arm.sim.domain.arm_model import BOARD, CLEARANCE, CUBE, GRIPPER, SERVOS
from roboter_arm.sim.domain.collision import arm_shapes, find_contacts, on_board
from roboter_arm.sim.domain.kinematics import (
    DEG, angles_from_counts, apply3, counts_for_gap, fk, gap_for_theta, gripper_theta,
    pad_depth_for_theta, point, theta_for_gap, tool_yaw_and_tilt, transpose3, wrap90)

COLOURS = ('orange', 'blue', 'yellow', 'green', 'red')
MAX_EVENTS = 300


@dataclass
class Cube:
    id: str
    colour: str
    p: list                  # centre, cm
    yaw: float = 0.0         # degrees, upright cube
    R: tuple | None = None   # set while held (follows the hand)
    held: bool = False

    def json(self):
        return {'id': self.id, 'colour': self.colour, 'p': list(self.p), 'yaw': self.yaw,
                'R': self.R, 'held': self.held, 'level': round((self.p[2] - CUBE / 2) / CUBE)}


@dataclass
class Holding:
    id: str
    width: float
    w: float                 # cube centre in the hand frame: jaw depth axis
    v: float                 # and along the tool


@dataclass
class World:
    servos: tuple = SERVOS
    cubes: list = field(default_factory=list)
    holding: Holding | None = None
    gap: float = 4.5
    contacts: list = field(default_factory=list)
    events: list = field(default_factory=list)
    seq: int = 0
    frames: object = None

    def copy(self):
        return copy.deepcopy(self)

    def event(self, kind, text):
        self.seq += 1
        self.events.append({'seq': self.seq, 'kind': kind, 'text': text})
        del self.events[:-MAX_EVENTS]

    def cube(self, cube_id):
        return next((c for c in self.cubes if c.id == cube_id), None)

    # ---------------- scene ----------------

    def set_scene(self, cubes):
        """[(colour, x, y, level, yaw), ...]; ids are the colour, numbered when repeated."""
        scene = []
        for entry in cubes:
            colour, x, y, level, yaw = (list(entry) + [0, 0])[:5]
            if colour not in COLOURS or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, level, yaw)):
                raise ValueError(f'Cube needs a colour of {", ".join(COLOURS)} and finite x, y, level, yaw')
            if not on_board(BOARD, x, y) or not 0 <= level <= 6:
                raise ValueError(f'{colour} cube at {x:.1f}, {y:.1f} is off the board')
            cube_id, n = colour, 2
            while any(c.id == cube_id for c in scene):
                cube_id, n = f'{colour}{n}', n + 1
            scene.append(Cube(cube_id, colour, [x, y, CUBE / 2 + int(level) * CUBE], float(yaw)))
        self.cubes, self.holding = scene, None
        self.settle()
        self.event('scene', f'scene with {len(scene)} cubes')

    @staticmethod
    def in_footprint(c, x, y, margin=0.0):
        a = -c.yaw * math.pi / 180
        dx, dy = x - c.p[0], y - c.p[1]
        lx, ly = dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)
        return abs(lx) <= CUBE / 2 + margin and abs(ly) <= CUBE / 2 + margin

    def support_for(self, probe):
        """Height a cube dropped at probe's centre lands on, what it rests on, and a cube whose
        edge it would sit on (no physics: it is placed level, with a warning)."""
        x, y = probe.p[0], probe.p[1]
        z, on, edge = (0.0, 'board', None) if on_board(BOARD, x, y) else (-BOARD['t'], 'floor', None)
        for o in self.cubes:
            if o is probe or o.held:
                continue
            top = o.p[2] + CUBE / 2
            if top > probe.p[2] - CUBE / 2 + 0.3:          # beside or above it
                continue
            if self.in_footprint(o, x, y) and top > z:
                z, on, edge = top, o.id, None
            elif self.in_footprint(o, x, y, CUBE * 0.5) and top > z:
                edge = o
        return z, on, edge

    def settle(self):
        """Drop every free cube onto its support, lowest first."""
        for c in sorted((c for c in self.cubes if not c.held), key=lambda c: c.p[2]):
            z, on, edge = self.support_for(c)
            if abs(c.p[2] - (z + CUBE / 2)) > 1e-6:
                c.p[2] = z + CUBE / 2
                if edge is not None:
                    self.event('warn', f'{c.id} lands on the edge of {edge.id}; a real cube would topple')

    # ---------------- one step ----------------

    def step(self, counts):
        """Arm at `counts` (6 values): grasp or release, move the held cube, report contacts."""
        f = self.frames = fk(angles_from_counts(counts, self.servos))
        cmd_gap = gap_for_theta(gripper_theta(counts[5], self.servos[5]))
        jaw_yaw, tilt = tool_yaw_and_tilt(f.hand)
        RT = transpose3(f.hand.R)
        local = lambda p: apply3(RT, (p[0] - f.hand.p[0], p[1] - f.hand.p[1], p[2] - f.hand.p[2]))

        if self.holding:
            c = self.cube(self.holding.id)
            if cmd_gap > self.holding.width + 0.02:
                self.holding, self.gap = None, cmd_gap
                c.held = False
                x = (c.R[0], c.R[3], c.R[6])
                c.yaw, c.R = math.atan2(x[1], x[0]) * DEG, None
                if tilt > 10:
                    self.event('warn', f'{c.id} released with the gripper {tilt:.0f}° off vertical')
                self.settle()
                self.event('release', f'released {c.id} on level {round((c.p[2] - CUBE / 2) / CUBE)}')
            else:
                self.gap = self.holding.width
        else:
            grasped = None
            v_pad = pad_depth_for_theta(theta_for_gap(max(cmd_gap, 0)))
            if cmd_gap < self.gap + 1e-9:         # the jaws close (or hold still)
                for c in self.cubes:
                    q = local(c.p)                 # u = jaw axis, w = depth, v = down the tool
                    e = wrap90(c.yaw - jaw_yaw)
                    width = CUBE if abs(e) < 20 else CUBE * (abs(math.cos(e * math.pi / 180)) + abs(math.sin(e * math.pi / 180)))
                    if cmd_gap >= width:
                        continue
                    between = abs(q[0]) + width / 2 <= self.gap / 2 + 0.05
                    v_overlap = abs(q[2] - v_pad) < CUBE / 2 + GRIPPER['pad']['h'] / 2 - 0.2
                    w_overlap = abs(q[1]) < CUBE / 2 + GRIPPER['pad']['w'] / 2 - 0.2
                    if not (between and v_overlap and w_overlap and tilt < 30):
                        continue
                    if abs(e) >= 20:
                        self.event('warn', f'jaws hit the corners of {c.id} ({e:.0f}° off), no grasp')
                        continue
                    grasped = c, width, q, e
                    break
            if grasped:
                c, width, q, e = grasped
                self.holding = Holding(c.id, width, q[1], q[2])
                c.held, self.gap = True, width
                above = [o.id for o in self.cubes if o is not c and not o.held and o.p[2] > c.p[2] + 0.3
                         and self.in_footprint(c, o.p[0], o.p[1], 0.3)]
                self.event('grasp', f'grasped {c.id}: centred by {abs(q[0]):.2f} cm, squared by {abs(e):.1f}°, '
                           f'jaws touch at S6 ≈ {counts_for_gap(width, self.servos[5]):.0f}'
                           + (f' ({", ".join(above)} on it will fall)' if above else ''))
            else:
                self.gap = cmd_gap

        if self.holding:
            c = self.cube(self.holding.id)
            c.p = list(point(f.hand, (0, self.holding.w, self.holding.v)))
            c.R = f.hand.R
            self.settle()
        self.contacts = find_contacts(arm_shapes(f, self.gap), self.cubes, BOARD, CLEARANCE)
        return self.contacts
