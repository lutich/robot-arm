"""MuJoCo physics behind the servo driver: targets in counts, stepping, poses and a sim clock."""
import math
import threading
import time

import mujoco
import numpy as np

from armsim.calibration import Calibration
from armsim.config import load_robot, load_scene
from armsim.model_builder import build_xml

CHANNELS = range(6)
JOINT_NAMES = ['S1', 'S2', 'S3', 'S4', 'S5', 'S6']


class Physics:
    """Owns model and data. Every public method takes the lock, so threads may share it."""

    def __init__(self, robot=None, scene=None, cameras=None):
        self.robot = robot or load_robot()
        self.scene = scene or load_scene()
        self.calibration = Calibration(self.robot)
        self.xml = build_xml(self.robot, self.scene, cameras)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.actuators = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'act_{name}') for name in JOINT_NAMES]
        self.joints = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
        self.gains = {a: (self.model.actuator_gainprm[a].copy(), self.model.actuator_biasprm[a].copy()) for a in self.actuators}
        self.enabled = {c: False for c in CHANNELS}
        self.targets = {c: None for c in CHANNELS}
        self.blocks = [b['name'] for b in self.scene['blocks']]
        self.reset()

    # ---- pose ---------------------------------------------------------------------------
    def reset(self, counts=None):
        """Scene back to its start layout; the arm starts limp at `counts` if given."""
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            for c in CHANNELS:
                self.enabled[c] = False
                self.targets[c] = None
                self._apply_gain(c)
            if counts is not None:
                self.place_arm(counts)
            mujoco.mj_forward(self.model, self.data)

    def place_arm(self, counts):
        """Teleport the joints to the angles of these counts, without actuating them."""
        with self.lock:
            for c, count in counts.items():
                angle = self.calibration.radians(c, count)
                self.data.qpos[self.model.jnt_qposadr[self.joints[c]]] = angle
                if c == 5:
                    for name in ('S6_right',):
                        self.data.qpos[self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]] = angle
                    for name in ('finger_left', 'finger_right'):
                        self.data.qpos[self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]] = -angle
            self.data.qvel[:] = 0
            mujoco.mj_forward(self.model, self.data)

    def set_target(self, channel, count):
        with self.lock:
            self.targets[channel] = count
            self.data.ctrl[self.actuators[channel]] = self.calibration.radians(channel, count)

    def enable(self, channel, enabled=True):
        with self.lock:
            self.enabled[channel] = enabled
            self._apply_gain(channel)

    def _apply_gain(self, channel):
        actuator = self.actuators[channel]
        gain, bias = self.gains[actuator]
        if self.enabled[channel]:
            self.model.actuator_gainprm[actuator] = gain
            self.model.actuator_biasprm[actuator] = bias
        else:
            self.model.actuator_gainprm[actuator] = 0
            self.model.actuator_biasprm[actuator] = 0

    # ---- stepping -----------------------------------------------------------------------
    def step(self, seconds):
        with self.lock:
            steps = max(1, int(round(seconds / self.model.opt.timestep)))
            for _ in range(steps):
                mujoco.mj_step(self.model, self.data)

    @property
    def time(self):
        return float(self.data.time)

    # ---- readings -----------------------------------------------------------------------
    def joint_angle(self, channel):
        """Current joint angle in degrees."""
        with self.lock:
            return math.degrees(self.data.qpos[self.model.jnt_qposadr[self.joints[channel]]])

    def joint_angles(self):
        return {c: self.joint_angle(c) for c in CHANNELS}

    def site(self, name):
        with self.lock:
            return self.data.site(name).xpos.copy()

    def body_pose(self, name):
        """Position and yaw (degrees) of a body."""
        with self.lock:
            pos = self.data.body(name).xpos.copy()
            w, x, y, z = self.data.body(name).xquat
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        return pos, yaw

    def body_up(self, name):
        """Z component of the body's local Z axis: 1 means upright."""
        with self.lock:
            return float(self.data.body(name).xmat.reshape(3, 3)[2, 2])

    def block_poses(self):
        return {name: self.body_pose(name) for name in self.blocks}

    def place_block(self, name, x, y, yaw=0.0, z=None):
        with self.lock:
            joint = self.model.joint(f'{name}_free')
            adr = joint.qposadr[0]
            spec = next(b for b in self.scene['blocks'] if b['name'] == name)
            if z is None:
                z = spec['size'] / 2 if spec['kind'] == 'cube' else 0.0
            half = math.radians(yaw) / 2
            self.data.qpos[adr:adr + 7] = [x, y, z, math.cos(half), 0, 0, math.sin(half)]
            self.data.qvel[joint.dofadr[0]:joint.dofadr[0] + 6] = 0
            mujoco.mj_forward(self.model, self.data)

    def pad_gap(self):
        left, right = self.site('pad_left'), self.site('pad_right')
        return float(np.linalg.norm(left - right))

    def grasp_point(self):
        return (self.site('pad_left') + self.site('pad_right')) / 2

    def contacts(self, geom_name):
        """Names of geoms touching the named geom."""
        with self.lock:
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            names = set()
            for i in range(self.data.ncon):
                con = self.data.contact[i]
                if gid in (con.geom1, con.geom2):
                    other = con.geom2 if con.geom1 == gid else con.geom1
                    names.add(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, other))
            return names


class SimClock:
    """Clock and sleep for robot-arm's Session: sleeping advances the physics instead of waiting."""

    def __init__(self, physics):
        self.physics = physics

    def clock(self):
        return self.physics.time

    def sleep(self, seconds):
        if seconds > 0:
            self.physics.step(seconds)


class RealtimeStepper:
    """Steps the physics in wall-clock time on a background thread."""

    def __init__(self, physics, slice_seconds=0.01):
        self.physics, self.slice = physics, slice_seconds
        self.finished = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, name='physics', daemon=True)
        self.thread.start()

    def _run(self):
        last = time.monotonic()
        while not self.finished.is_set():
            now = time.monotonic()
            elapsed = min(now - last, 0.1)
            last = now
            if elapsed > 0:
                self.physics.step(elapsed)
            self.finished.wait(self.slice)

    def stop(self):
        self.finished.set()
        if self.thread is not None:
            self.thread.join(timeout=2)


def _layout_pose(scene, name):
    spec = next(b for b in scene['blocks'] if b['name'] == name)
    slot = scene['slots'][scene['layout'][name]]
    return spec, slot


def reset_blocks(physics):
    """Blocks back to their start slots; the arm keeps its state."""
    for name in physics.blocks:
        spec, slot = _layout_pose(physics.scene, name)
        physics.place_block(name, slot['x'], slot['y'], slot.get('yaw', 0))
