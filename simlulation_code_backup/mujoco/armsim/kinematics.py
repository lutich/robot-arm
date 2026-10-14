"""Forward and inverse kinematics of the arm in the table frame, with counts at both ends.

Angle conventions (degrees): S1 yaw counter-clockwise from +X seen from above; S2 lean forward
from vertical; S3 bend relative to the upper arm; S4 flex relative to the forearm; S5 roll.
The gripper points straight down when S2 + S3 + S4 = 180.
"""
import math

import numpy as np

from armsim.calibration import Calibration

DOWN = 180.0


class Kinematics:
    def __init__(self, robot, scene, bounds):
        links, g = robot['links'], robot['gripper']
        self.h0, self.l1, self.l2, self.l3 = links['shoulder_height'], links['upper_arm'], links['forearm'], links['wrist']
        self.base = np.array([scene['base']['x'], scene['base']['y'], 0.0])
        self.g = g
        self.calibration = Calibration(robot)
        self.bounds = bounds

    # ---- gripper ------------------------------------------------------------------------
    def link_angle_for_gap(self, gap):
        """Gear link angle (degrees) at which the pads are `gap` apart."""
        s = (gap / 2 + self.g['tip_inward'] - self.g['gear_u']) / self.g['link_length']
        if not -1 <= s <= 1:
            raise ValueError('Gap is outside the gripper range')
        return math.degrees(math.asin(s))

    def gripper_length(self, gap=None, angle=None):
        """Flange to pad centre, for a pad gap or a link angle."""
        if angle is None:
            angle = self.link_angle_for_gap(gap)
        return self.g['gear_v'] + self.g['link_length'] * math.cos(math.radians(angle)) + self.g['tip_down']

    def gripper_count_for_gap(self, gap):
        return self.calibration.count(5, self.link_angle_for_gap(gap))

    # ---- forward ------------------------------------------------------------------------
    def forward(self, angles):
        """Joint positions for angles {0..5}: s2, s3, s4, flange, grasp point, and yaw."""
        a1, a2, a3 = (math.radians(angles[c]) for c in (1, 2, 3))
        yaw = math.radians(angles[0])
        radial = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        s2 = self.base + up * self.h0
        s3 = s2 + self.l1 * (math.sin(a1) * radial + math.cos(a1) * up)
        s4 = s3 + self.l2 * (math.sin(a1 + a2) * radial + math.cos(a1 + a2) * up)
        direction = math.sin(a1 + a2 + a3) * radial + math.cos(a1 + a2 + a3) * up
        flange = s4 + self.l3 * direction
        grasp = flange + self.gripper_length(angle=angles[5]) * direction
        return dict(s2=s2, s3=s3, s4=s4, flange=flange, grasp=grasp, direction=direction)

    def forward_counts(self, counts):
        return self.forward(self.calibration.angles(counts))

    # ---- inverse ------------------------------------------------------------------------
    def roll_options(self, arm_yaw, block_yaw, symmetry=90):
        """Rolls that put the pads on the gripped faces, nearest to zero first.

        At zero roll the pads separate along the arm's radial direction (pads_axis x) or sideways
        (pads_axis y); a positive roll turns them clockwise seen from above. The gripped faces have
        the normal block_yaw + 90 (the block's local y). Cubes repeat every 90°, longer blocks 180°.
        """
        offset = 90 if self.g.get('pads_axis', 'x') == 'y' else 0     # pad axis at zero roll: sideways or radial
        base = arm_yaw + offset - block_yaw - 90
        options = sorted({((base + k * symmetry + 180) % 360) - 180 for k in range(0, 360 // symmetry)}, key=abs)
        return options

    def inverse(self, point, gap, block_yaw=0.0, symmetry=90, roll=None):
        """Angles that put the grasp point at `point` with the gripper straight down and pads `gap` apart."""
        x, y, z = (float(v) for v in point)
        dx, dy = x - self.base[0], y - self.base[1]
        yaw = math.degrees(math.atan2(dy, dx))
        r = math.hypot(dx, dy)
        link_angle = self.link_angle_for_gap(gap)
        z4 = z + self.gripper_length(angle=link_angle) + self.l3
        px, pz = r, z4 - self.h0
        d = math.hypot(px, pz)
        if d > self.l1 + self.l2 - 1e-6 or d < abs(self.l1 - self.l2) + 1e-6:
            raise ValueError('Target is out of reach')
        cos_a2 = (d * d - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        a2 = math.degrees(math.acos(max(-1.0, min(1.0, cos_a2))))      # elbow bends forward and down
        a1 = math.degrees(math.atan2(px, pz) - math.atan2(self.l2 * math.sin(math.radians(a2)), self.l1 + self.l2 * math.cos(math.radians(a2))))
        a3 = DOWN - a1 - a2
        if roll is None:
            roll = self.roll_options(yaw, block_yaw, symmetry)[0]
        return {0: yaw, 1: a1, 2: a2, 3: a3, 4: roll, 5: link_angle}

    def inverse_counts(self, point, gap, block_yaw=0.0, symmetry=90):
        """Counts for the pose; among the equivalent rolls, the first one inside the servo bounds."""
        angles = self.inverse(point, gap, block_yaw, symmetry)
        failure = None
        for roll in self.roll_options(angles[0], block_yaw, symmetry):
            chosen = dict(angles)
            chosen[4] = roll
            counts = {c: self.calibration.count(c, a) for c, a in chosen.items()}
            try:
                for c, count in counts.items():
                    low, high = self.bounds[c]
                    if not low <= count <= high:
                        raise ValueError(f'Channel {c} needs count {count}, outside its bounds {low}–{high}')
                return counts
            except ValueError as error:
                failure = error
        raise failure
