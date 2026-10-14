"""Pick and place in counts: every step is a full six-count pose sent through an ArmClient.

The task layer needs block positions from a World: the in-process physics, or the sandbox's
/sim/blocks route over HTTP. On the real arm these would come from the camera.
"""
import json
import time

from armsim.kinematics import Kinematics

OPEN_GAP = 0.045
SQUEEZE = 0.004          # close to this much below the block width so the servo presses on it


class PhysicsWorld:
    def __init__(self, physics, scene):
        self.physics, self.scene = physics, scene

    def blocks(self):
        specs = {b['name']: b for b in self.scene['blocks']}
        result = {}
        for name, (pos, yaw) in self.physics.block_poses().items():
            result[name] = _block_info(specs[name], pos, yaw)
        return result

    def settle(self, seconds=0.5):
        self.physics.step(seconds)


class HttpWorld:
    def __init__(self, client):
        self.client = client

    def blocks(self):
        return {name: dict(info) for name, info in self.client.sim('blocks').items()}

    def settle(self, seconds=0.5):
        time.sleep(seconds)


class ManualWorld:
    """Block positions typed in by the operator, for a target without /sim/blocks: the real arm.

    Positions are in the table frame of the printed reference sheet, metres, yaw in degrees.
    The task layer updates them after every set-down, so a sequence of picks stays consistent.
    """

    def __init__(self, scene):
        self.scene = scene
        self.specs = {b['name']: b for b in scene['blocks']}
        self.poses = {}

    def set(self, name, x, y, yaw=0.0, centre_z=None):
        spec = self.specs[name]
        height = spec['size'] if spec['kind'] == 'cube' else spec['height']
        self.poses[name] = (float(x), float(y), height / 2 if centre_z is None else float(centre_z), float(yaw))

    def blocks(self):
        result = {}
        for name, (x, y, centre, yaw) in self.poses.items():
            spec = self.specs[name]
            z = centre if spec['kind'] == 'cube' else centre - spec['height'] / 2
            result[name] = _block_info(spec, (x, y, z), yaw)
        return result

    def moved(self, name, x, y, centre_z, yaw):
        self.poses[name] = (x, y, centre_z, yaw)

    def settle(self, seconds=0.5):
        time.sleep(seconds)


def _block_info(spec, pos, yaw):
    if spec['kind'] == 'cube':
        width, height, centre = spec['size'], spec['size'], float(pos[2])
    else:
        width, height, centre = spec['depth'], spec['height'], float(pos[2]) + spec['height'] / 2
    return dict(x=float(pos[0]), y=float(pos[1]), z=centre, yaw=float(yaw), width=width, height=height, kind=spec['kind'],
                symmetry=90 if spec['kind'] == 'cube' else 180)


class PickPlace:
    def __init__(self, client, world, robot, scene, bounds, log=None):
        self.client, self.world = client, world
        self.scene, self.kin = scene, Kinematics(robot, scene, bounds)
        self.task = scene['task']
        self.log = log if log is not None else []
        self.held = None

    # ---- poses ----------------------------------------------------------------------
    def _pose(self, point, gap, yaw, symmetry=90):
        return self.kin.inverse_counts(point, gap, yaw, symmetry)

    def _go(self, label, counts, speed):
        self.log.append(dict(label=label, counts=[int(counts[c]) for c in range(6)], speed=int(speed), t=time.time()))
        self.client.move_pose(counts, speed=speed)

    def _grip(self, label, gap, pose):
        counts = dict(pose)
        counts[5] = self.kin.gripper_count_for_gap(gap)
        self._go(label, counts, self.task['approach_speed'])
        self.world.settle(0.5)
        return counts

    def _travel_height(self):
        return self.scene['slots'][next(iter(self.scene['slots']))].get('z', 0) + 0.0265 / 2 + self.task['safe_height']

    def _rise(self, label):
        """Lift straight up from the current pose to the travel height before moving sideways."""
        current = self.client.commands()
        if any(current.get(c) is None for c in range(6)):
            raise ValueError('All six commands must be known')
        fk = self.kin.forward_counts(current)
        gap = self.held['width'] if self.held else OPEN_GAP
        point = (fk['grasp'][0], fk['grasp'][1], max(fk['grasp'][2], self._travel_height()))
        try:
            pose = self.kin.inverse_counts(point, gap, 0.0)
        except ValueError:
            return                      # already high, or the lift is unreachable from here
        pose[4], pose[5] = current[4], current[5]
        if pose != current:
            self._go(label, pose, self.task['approach_speed'])

    # ---- actions --------------------------------------------------------------------
    def pick(self, name):
        block = self.world.blocks()[name]
        yaw, gap_open, sym = block['yaw'], OPEN_GAP, block['symmetry']
        centre = (block['x'], block['y'], block['z'])
        above = (block['x'], block['y'], block['z'] + self.task['safe_height'])
        open_count = self.kin.gripper_count_for_gap(gap_open)
        self._rise('rise before pick')
        self._go(f'above {name}', self._pose(above, gap_open, yaw, sym), self.task['travel_speed'])
        # The pads move down as the gripper closes, so descend with the flange height of the
        # held state; the open pads then sit a little above the block centre and close onto it.
        descend = self._pose(centre, block['width'], yaw, sym)
        descend[5] = open_count
        self._go(f'descend to {name}', descend, self.task['approach_speed'])
        gap_hold = max(block['width'] - SQUEEZE, 0.0)
        held_pose = self._grip(f'close on {name}', gap_hold, descend)
        lift = self.kin.inverse_counts(above, block['width'], yaw, sym)
        lift[5] = held_pose[5]
        self._go(f'lift {name}', lift, self.task['approach_speed'])
        self.held = dict(block, name=name, gap=gap_hold)
        return self.held

    def place_on(self, base_name):
        """Set the held block down on top of another block."""
        if self.held is None:
            raise ValueError('Nothing is held')
        block = self.held
        base = self.world.blocks()[base_name]
        if base_name == block['name']:
            raise ValueError('A block cannot be stacked on itself')
        top = base['z'] + base['height'] / 2
        self._set_down(block, (base['x'], base['y']), top + block['height'] / 2, base['yaw'], f'on {base_name}')

    def place(self, slot_name):
        if self.held is None:
            raise ValueError('Nothing is held')
        block, slot = self.held, self.scene['slots'][slot_name]
        self._set_down(block, (slot['x'], slot['y']), block['z'], slot.get('yaw', 0), slot_name)

    def _set_down(self, block, xy, centre_z, yaw, where):
        sym = block['symmetry']
        target = (xy[0], xy[1], centre_z + 0.002)
        above = (xy[0], xy[1], centre_z + self.task['safe_height'])
        width = block['width']
        travel = self.kin.inverse_counts(above, width, yaw, sym)
        travel[5] = self.kin.gripper_count_for_gap(block['gap'])
        self._go(f'carry {block["name"]} above {where}', travel, self.task['travel_speed'])
        lower = self.kin.inverse_counts(target, width, yaw, sym)
        lower[5] = travel[5]
        self._go(f'lower {block["name"]} {where}', lower, self.task['approach_speed'])
        self._grip(f'release {block["name"]}', OPEN_GAP, lower)
        self._go(f'retreat from {where}', self._pose(above, OPEN_GAP, yaw, sym), self.task['approach_speed'])
        if hasattr(self.world, 'moved'):
            self.world.moved(block['name'], xy[0], xy[1], centre_z, yaw)
        self.held = None

    def stack(self, name, base_name):
        self.pick(name)
        self.place_on(base_name)
        self.home()

    def home(self):
        self._rise('rise before home')
        home = {j['channel']: j['home'] for j in self.client.state()['joints']}     # the session's active HOME
        fk = self.kin.forward_counts(home)
        above_home = (fk['grasp'][0], fk['grasp'][1], max(fk['grasp'][2], self._travel_height()))
        try:
            pose = self.kin.inverse_counts(above_home, OPEN_GAP, 0.0)
            pose[4] = self.client.commands()[4]
            self._go('above home', pose, self.task['travel_speed'])
        except ValueError:
            pass
        self.client.go_pose('HOME')
        self.client.wait_idle()
        self.log.append(dict(label='home', counts=[self.client.commands()[c] for c in range(6)], speed=20, t=time.time()))

    def run(self, name, slot_name):
        self.pick(name)
        self.place(slot_name)
        self.home()

    def save_log(self, path):
        with open(path, 'w') as stream:
            for entry in self.log:
                stream.write(json.dumps(entry) + '\n')
