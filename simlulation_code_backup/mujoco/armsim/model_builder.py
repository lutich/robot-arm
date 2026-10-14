"""Generates the MuJoCo model (MJCF) from config/robot.yaml and config/scene.yaml.

Gripper jaws open in the arm plane (gripper.pads_axis: x) or across it (y) at S5 home.
Table frame: origin at the midpoint of the table edge nearest the arm, +X away from the arm,
+Y left seen from the arm, Z up. Pitch joints turn about +Y, so a positive angle leans a link
toward +X. Joint names S1..S6 match the servo channels 0..5.
"""
import math

from armsim.calibration import Calibration
from armsim.robotarm import BOUNDS, PARK

ARM = 'class="arm"'
OBJECT = 'class="object"'
VISUAL = 'class="visual"'
PAD = 'class="pad"'


def fmt(*values):
    return ' '.join(f'{v:.6g}' for v in values)


def rgba(color):
    return fmt(*color)


def camera_xml(name, spec, width, height):
    """A MuJoCo camera from a lookat/azimuth/elevation spec or an explicit pos/xyaxes spec."""
    if 'pos' in spec:
        extra = ''
        if 'focalpixel' in spec:
            extra = (f' focalpixel="{fmt(*spec["focalpixel"])}" principalpixel="{fmt(*spec["principalpixel"])}"'
                     f' sensorsize="{fmt(width / 1000, height / 1000)}" resolution="{width} {height}"')
        else:
            extra = f' fovy="{spec.get("fovy", 45)}"'
        return f'<camera name="{name}" pos="{fmt(*spec["pos"])}" xyaxes="{fmt(*spec["xyaxes"])}"{extra}/>'
    az, el = math.radians(spec['azimuth']), math.radians(spec['elevation'])
    forward = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
    lookat = spec['lookat']
    pos = tuple(lookat[i] - spec['distance'] * forward[i] for i in range(3))
    right = _normalize(_cross(forward, (0, 0, 1)))
    up = _cross(right, forward)
    return f'<camera name="{name}" pos="{fmt(*pos)}" xyaxes="{fmt(*right, *up)}" fovy="{spec.get("fovy", 45)}"/>'


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _normalize(v):
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return tuple(x / n for x in v)


def yaw_quat(yaw_deg):
    half = math.radians(yaw_deg) / 2
    return fmt(math.cos(half), 0, 0, math.sin(half))


def halfround_vertices(width, height, depth, segments=24):
    """Convex extruded circular segment, flat side at z=0, arc up to z=height."""
    radius = (width ** 2 / 4 + height ** 2) / (2 * height)
    phi0 = math.asin(min(1.0, (width / 2) / radius))
    centre_z = height - radius
    points = [(-width / 2, 0.0), (width / 2, 0.0)]
    for i in range(segments + 1):
        phi = -phi0 + 2 * phi0 * i / segments
        points.append((radius * math.sin(phi), centre_z + radius * math.cos(phi)))
    vertices = []
    for y in (-depth / 2, depth / 2):
        for x, z in points:
            vertices.extend((x, y, max(0.0, z)))
    return vertices


def block_xml(block, slot):
    pos_x, pos_y, yaw = slot['x'], slot['y'], slot.get('yaw', 0)
    color = rgba(block['color'])
    if block['kind'] == 'cube':
        half = block['size'] / 2
        return (f'<body name="{block["name"]}" pos="{fmt(pos_x, pos_y, half)}" quat="{yaw_quat(yaw)}"><freejoint name="{block["name"]}_free"/>'
                f'<geom name="{block["name"]}" type="box" size="{fmt(half, half, half)}" mass="{block["mass"]}" rgba="{color}" {OBJECT}/></body>')
    if block['kind'] == 'halfround':
        return (f'<body name="{block["name"]}" pos="{fmt(pos_x, pos_y, 0)}" quat="{yaw_quat(yaw)}"><freejoint name="{block["name"]}_free"/>'
                f'<geom name="{block["name"]}" type="mesh" mesh="{block["name"]}_mesh" mass="{block["mass"]}" rgba="{color}" {OBJECT}/></body>')
    raise ValueError(f'unknown block kind {block["kind"]}')


def rest_height(robot, cal, inline, park=None):
    """Lowest point of the pads in the PARK pose (metres above the table), or 0 if below it."""
    park = park or PARK
    links, g = robot['links'], robot['gripper']
    a1, a2, a3 = (math.radians(cal.angle(c, park[c])) for c in (1, 2, 3))
    link = math.radians(cal.angle(5, park[5]))
    grasp_z = (links['shoulder_height'] + links['upper_arm'] * math.cos(a1) + links['forearm'] * math.cos(a1 + a2)
               + (links['wrist'] + g['gear_v'] + g['link_length'] * math.cos(link) + g['tip_down']) * math.cos(a1 + a2 + a3))
    tilt = a1 + a2 + a3 - math.pi                     # gripper tilt from straight down, in the arm plane
    half_gap = g['gear_u'] + g['link_length'] * math.sin(link) - g['tip_inward']
    drop = abs(half_gap * math.sin(tilt)) if inline else 0.0
    lowest = grasp_z - drop - g['pad_height'] / 2 * abs(math.cos(tilt)) - g['pad_thickness'] / 2 * abs(math.sin(tilt))
    return max(0.0, lowest - 0.0003)


def build_xml(robot, scene, cameras=None):
    links, gripper, servo = robot['links'], robot['gripper'], robot['servo']
    table, base, physics = scene['table'], scene['base'], scene['physics']
    render = scene.get('render', {})
    width, height = int(render.get('width', 1280)), int(render.get('height', 720))
    h0, l1, l2, l3 = links['shoulder_height'], links['upper_arm'], links['forearm'], links['wrist']
    sb = servo['body']
    link_mass = robot.get('link_mass', 0.03)
    kp, kv, torque = servo['kp'], servo['kv'], servo['torque']
    g_l, g_u, g_v = gripper['link_length'], gripper['gear_u'], gripper['gear_v']
    tip_in, tip_down = gripper['tip_inward'], gripper['tip_down']
    pad_h, pad_t = gripper['pad_height'], gripper['pad_thickness']

    inline = gripper.get('pads_axis', 'x') == 'x'      # jaws open in the arm plane (x) or across it (y)

    def jaw(side):
        s = -1 if side == 'left' else 1
        if inline:
            pivot, axis, tip = (s * g_u, 0, g_v), fmt(0, s, 0), (-s * tip_in, 0, tip_down)
            pad_pos, pad_size = (tip[0] + s * pad_t / 2, 0, tip_down), (pad_t / 2, 0.008, pad_h / 2)
            finger_end = (tip[0] + s * 0.005, 0, tip_down)
        else:
            pivot, axis, tip = (0, s * g_u, g_v), fmt(-s, 0, 0), (0, -s * tip_in, tip_down)
            pad_pos, pad_size = (0, tip[1] + s * pad_t / 2, tip_down), (0.008, pad_t / 2, pad_h / 2)
            finger_end = (0, tip[1] + s * 0.005, tip_down)
        return (f'<body name="jaw_{side}" pos="{fmt(*pivot)}">'
                f'<joint name="{"S6" if side == "left" else "S6_right"}" type="hinge" axis="{axis}" damping="0.002" range="{fmt(g_lo - 3, g_hi + 3)}" frictionloss="0.005"/>'
                f'<geom name="link_{side}" type="capsule" fromto="0 0 0 0 0 {g_l}" size="0.004" mass="0.004" rgba="0.14 0.16 0.19 1" {ARM}/>'
                f'<body name="finger_{side}" pos="{fmt(0, 0, g_l)}">'
                f'<joint name="finger_{side}" type="hinge" axis="{axis}" damping="0.002" range="{fmt(-g_hi - 3, -g_lo + 3)}"/>'
                f'<geom name="finger_{side}" type="capsule" fromto="0 0 0 {fmt(*finger_end)}" size="0.004" mass="0.006" rgba="0.14 0.16 0.19 1" {ARM}/>'
                f'<geom name="pad_{side}" type="box" pos="{fmt(*pad_pos)}" size="{fmt(*pad_size)}" mass="0.001" rgba="0.9 0.9 0.9 1" {PAD} friction="{fmt(*physics["friction"])}"/>'
                f'<site name="pad_{side}" pos="{fmt(*tip)}" size="0.002" rgba="1 0 0 0"/>'
                f'</body></body>')

    mesh_assets = ''.join(
        f'<mesh name="{b["name"]}_mesh" vertex="{fmt(*halfround_vertices(b["width"], b["height"], b["depth"]))}"/>'
        for b in scene['blocks'] if b['kind'] == 'halfround')
    slots = scene['slots']
    blocks = ''.join(block_xml(b, slots[scene['layout'][b['name']]]) for b in scene['blocks'])
    cams = cameras if cameras is not None else scene.get('cameras', {})
    camera_defs = ''.join(camera_xml(name, spec, width, height) for name, spec in cams.items())
    plate_w, plate_d, plate_t = base['plate']
    # Joint stops from the servo count bounds, so an unpowered arm cannot spin or fold freely,
    # and gear friction so it sags slowly instead of dropping, as the geared MG996R does.
    cal = Calibration(robot)
    friction = servo.get('gear_friction', 0.1)

    def limits(channel):
        lo, hi = sorted(cal.angle(channel, b) for b in BOUNDS[channel])
        return f'range="{fmt(lo - 2, hi + 2)}" frictionloss="{friction}"'
    g_lo, g_hi = sorted((gripper['closed_angle'], gripper['open_angle']))
    # The base plate is where the gripper rests in PARK. Its top is set to the lowest pad point of
    # the PARK pose, so the pads rest on it instead of pressing into it.
    plate_t = max(plate_t, rest_height(robot, cal, inline))

    return f'''<mujoco model="arm_sandbox">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="{physics['timestep']}" gravity="0 0 {physics['gravity']}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="{width}" offheight="{height}"/><map znear="0.005"/><quality shadowsize="2048"/></visual>
  <default>
    <geom condim="4" solref="0.005 1" solimp="0.95 0.99 0.001"/>
    <default class="arm"><geom contype="2" conaffinity="1" rgba="0.15 0.17 0.2 1"/></default>
    <default class="object"><geom contype="1" conaffinity="1" friction="{fmt(*physics['friction'])}"/></default>
    <default class="visual"><geom contype="0" conaffinity="0"/></default>
    <default class="pad"><geom contype="4" conaffinity="1"/></default>
    <default class="rest"><geom contype="0" conaffinity="4"/></default>
    <joint type="hinge" damping="0.05" armature="0.001"/>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.21 0.23" rgb2="0.17 0.18 0.2" width="300" height="300"/>
    <material name="table" texture="grid" texrepeat="38 30" reflectance="0.05"/>
    {mesh_assets}
  </asset>
  <worldbody>
    <light pos="0.2 0.3 1.2" dir="-0.1 -0.2 -1" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" castshadow="true"/>
    <light pos="0.6 -0.5 0.8" dir="-0.5 0.4 -0.8" diffuse="0.4 0.4 0.4" castshadow="false"/>
    <geom name="table" type="box" pos="{fmt(table['depth'] / 2, 0, -table['thickness'] / 2)}" size="{fmt(table['depth'] / 2, table['width'] / 2, table['thickness'] / 2)}" material="table" friction="{fmt(*physics['table_friction'])}" {OBJECT}/>
    <geom name="floor" type="plane" pos="0 0 {fmt(-table['thickness'])}" size="2 2 0.1" rgba="0.6 0.6 0.62 1" {VISUAL}/>
    {camera_defs}
    <body name="base" pos="{fmt(base['x'], base['y'], 0)}">
      <joint name="S1" axis="0 0 1" {limits(0)}/>
      <geom name="base_plate" type="box" pos="0 0 {fmt(plate_t / 2)}" size="{fmt(plate_w / 2, plate_d / 2, plate_t / 2)}" mass="0.05" rgba="0.1 0.1 0.12 1" class="rest"/>
      <geom name="column" type="box" pos="0 0 {fmt(h0 / 2)}" size="{fmt(0.016, 0.016, h0 / 2)}" mass="{link_mass}" {VISUAL}/>
      <geom name="servo_S1" type="box" pos="0 0 {fmt(plate_t + sb[2] / 2)}" size="{fmt(sb[0] / 2, sb[1] / 2, sb[2] / 2)}" mass="{servo['mass']}" rgba="0.12 0.12 0.14 1" {VISUAL}/>
      <body name="shoulder" pos="0 0 {fmt(h0)}">
        <joint name="S2" axis="0 1 0" {limits(1)}/>
        <site name="s2" size="0.003" rgba="0 0 1 0"/>
        <geom name="servo_S2" type="box" size="{fmt(sb[0] / 2, sb[1] / 2, sb[2] / 2)}" mass="{servo['mass']}" rgba="0.12 0.12 0.14 1" {VISUAL}/>
        <geom name="upper_arm" type="box" pos="0 0 {fmt(l1 / 2)}" size="{fmt(0.012, 0.02, l1 / 2)}" mass="{link_mass}" {ARM}/>
        <body name="elbow" pos="0 0 {fmt(l1)}">
          <joint name="S3" axis="0 1 0" {limits(2)}/>
          <site name="s3" size="0.003" rgba="0 0 1 0"/>
          <geom name="servo_S3" type="box" size="{fmt(sb[0] / 2, sb[1] / 2, sb[2] / 2)}" mass="{servo['mass']}" rgba="0.12 0.12 0.14 1" {VISUAL}/>
          <geom name="forearm" type="cylinder" fromto="0 0 0.022 0 0 {fmt(l2 - 0.02)}" size="0.0065" mass="{link_mass}" rgba="0.75 0.77 0.8 1" {ARM}/>
          <body name="wrist" pos="0 0 {fmt(l2)}">
            <joint name="S4" axis="0 1 0" {limits(3)}/>
            <site name="s4" size="0.003" rgba="0 0 1 0"/>
            <geom name="servo_S4" type="box" size="{fmt(sb[0] / 2, sb[1] / 2, sb[2] / 2)}" mass="{servo['mass']}" rgba="0.12 0.12 0.14 1" {VISUAL}/>
            <geom name="wrist_link" type="box" pos="0 0 {fmt(l3 / 2)}" size="{fmt(0.016, 0.011, l3 / 2)}" mass="{link_mass}" {ARM}/>
            <body name="flange" pos="0 0 {fmt(l3)}">
              <joint name="S5" axis="0 0 1" {limits(4)}/>
              <site name="flange" size="0.003" rgba="0 1 0 0"/>
              <geom name="gripper_base" type="box" pos="0 0 {fmt(gripper['base_plate_height'] / 2)}" size="{fmt(gripper['base_plate_width'] / 2, 0.006, gripper['base_plate_height'] / 2) if inline else fmt(0.006, gripper['base_plate_width'] / 2, gripper['base_plate_height'] / 2)}" mass="0.03" rgba="0.14 0.16 0.19 1" {ARM}/>
              {jaw('left')}
              {jaw('right')}
            </body>
          </body>
        </body>
      </body>
    </body>
    {blocks}
  </worldbody>
  <equality>
    <joint name="mirror" joint1="S6_right" joint2="S6" polycoef="0 1 0 0 0"/>
    <joint name="parallel_left" joint1="finger_left" joint2="S6" polycoef="0 -1 0 0 0"/>
    <joint name="parallel_right" joint1="finger_right" joint2="S6_right" polycoef="0 -1 0 0 0"/>
  </equality>
  <actuator>
    <position name="act_S1" joint="S1" kp="{kp}" kv="{kv}" forcerange="{fmt(-torque, torque)}"/>
    <position name="act_S2" joint="S2" kp="{kp}" kv="{kv}" forcerange="{fmt(-torque, torque)}"/>
    <position name="act_S3" joint="S3" kp="{kp}" kv="{kv}" forcerange="{fmt(-torque, torque)}"/>
    <position name="act_S4" joint="S4" kp="{kp}" kv="{kv}" forcerange="{fmt(-torque, torque)}"/>
    <position name="act_S5" joint="S5" kp="{kp}" kv="{kv}" forcerange="{fmt(-torque, torque)}"/>
    <position name="act_S6" joint="S6" kp="{kp / 3}" kv="{kv / 5}" forcerange="{fmt(-gripper['torque'], gripper['torque'])}"/>
  </actuator>
</mujoco>'''
