"""Dimensions and servo calibration of the arm model, read from arm_model.json next to this file.

That file is the only copy: the simulation uses it here and serves it to the viewer
(GET /api/sim/model).
Lengths in cm, angles in degrees, servo values in PCA9685 counts at 60 Hz.
Frame (px-robotic-arm-sim docs/arm-dimensions.md): origin on the board top under the S1 axis,
Z up, +Y from the back edge toward the front, +X to the right seen from the robot.

arm      H_S2 board top to S2 axis; L2 shoulder to elbow; L3 elbow to wrist flexion (S4);
         L4 wrist flexion to the gripper flange (S5); G_BLOCK flange to the centre of a held cube.
board    w x d, thickness t; S1 axis is axisToBackEdge in front of the back edge.
gripper  parallel linkage in gripper-local u (sideways) and v (down from the flange): gear centre,
         outer link pivot, link length, pad contact point relative to the gear-link end; link
         angle from vertical at S6 open (490) and closed (650). Reproduces the measured values:
         open 10,0 / gap 4,5; holding the cube 11,0 / gap 2,65; closed 11,5 / gap 0.
servos   `ref` is the count at which the model angle is `q_ref` (a calibration point, not the
         HOME pose); `span` is [counts, degrees]; direction +1 if more counts mean a larger angle.
         Model angles: 0 = all links straight up, positive S2..S4 tilt toward +Y, positive S1
         turns +Y toward +X. S5's reference 250 was the live controller's HOME on 2026-10-12.

clearance   collision tolerance, cm: penetration up to this depth is not a contact.
press_depth deepest press of the fingers on the board (or of a held cube) that still counts as
            pressing, cm. The team's saved demos press the fingers up to 15 mm (commanded) into
            the board; the board stops a real servo short of its command there. Deeper, it would
            stall hard against it.
rest_zone   counts per joint around PARK in which contacts are resting, not collisions: PARK
            rests the gripper on the base and the board, and leaving it the model's fingers clear
            the base only about 17 counts out; rounded one-joint-at-a-time writes touch it again.

q_ref and direction are NOT measured. They were fitted to the taught demo
"step1-hacks-10-direct-approach": its "Single approach" pose puts the cube centre at 1,3 cm
height, 19 cm out, gripper pointing down, and PARK rests the gripper on the board. The fitted
directions match the controller's historical sign table (S2 -1, S3 +1, S4 -1). Until they are
measured the model may only warn on hardware, never block.
Link lengths, gripper, board and cube are from the schematic. Servo body and bracket sizes
(collision.py) are only for drawing and collision, taken from photos and the MG996R datasheet.
"""
from dataclasses import dataclass
import json
from pathlib import Path

MODEL = json.loads(Path(__file__).with_name('arm_model.json').read_text())


@dataclass(frozen=True)
class Servo:
    ch: int
    name: str
    joint: str
    ref: int
    low: int
    high: int
    cpd: float | None = None      # counts per degree
    q_ref: float = 0.0
    direction: int = 1
    open: int | None = None       # gripper only
    closed: int | None = None


def _servo(entry):
    entry = dict(entry)
    span = entry.pop('span', None)
    return Servo(**entry, cpd=span[0] / span[1] if span else None)


ARM = MODEL['arm']
BOARD = MODEL['board']
CUBE = MODEL['cube']
GRIPPER = MODEL['gripper']
SERVOS = tuple(_servo(entry) for entry in MODEL['servos'])
CLEARANCE = MODEL['clearance']
PRESS_DEPTH = MODEL['press_depth']
REST_ZONE = MODEL['rest_zone']
