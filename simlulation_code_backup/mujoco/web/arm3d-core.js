// Pure functions behind the 3D view: counts <-> angles, forward kinematics, and a minimal
// block physics. No rendering, no dependencies; tested with node (tests/arm3d-core.test.js).
// Frame: table frame of config/scene.yaml, metres, Z up, X away from the arm, Y left.

const DEG = Math.PI / 180;

export function angleOf(config, channel, count) {
  if (channel === 5) {
    const g = config.gripper;
    const f = (count - g.closed_count) / (g.open_count - g.closed_count);
    return g.closed_angle + f * (g.open_angle - g.closed_angle);
  }
  const c = config.channels[String(channel)];
  return c.direction * (count - c.home_count) / c.counts_per_degree + c.home_angle;
}

export function countOf(config, channel, angle) {
  if (channel === 5) {
    const g = config.gripper;
    const f = (angle - g.closed_angle) / (g.open_angle - g.closed_angle);
    return Math.round(g.closed_count + f * (g.open_count - g.closed_count));
  }
  const c = config.channels[String(channel)];
  return Math.round(c.home_count + c.direction * (angle - c.home_angle) * c.counts_per_degree);
}

export function anglesOf(config, counts) {
  return counts.map((count, channel) => angleOf(config, channel, count));
}

export function linkAngleForGap(config, gap) {
  const g = config.gripper;
  const s = (gap / 2 + g.tip_inward - g.gear_u) / g.link_length;
  return Math.asin(Math.max(-1, Math.min(1, s))) / DEG;
}

export function gapForLinkAngle(config, angle) {
  const g = config.gripper;
  return 2 * (g.gear_u + g.link_length * Math.sin(angle * DEG) - g.tip_inward);
}

export function gripperLength(config, angle) {
  const g = config.gripper;
  return g.gear_v + g.link_length * Math.cos(angle * DEG) + g.tip_down;
}

// Joint positions for six angles in degrees. Pads separate along the sideways direction
// (arm yaw + 90°) turned clockwise by the roll, as in armsim/kinematics.py.
export function forward(config, angles) {
  const [yaw, a1, a2, a3, roll, link] = angles.map((a) => a * DEG);
  const L = config.links;
  const radial = [Math.cos(yaw), Math.sin(yaw), 0];
  const up = [0, 0, 1];
  const add = (p, v, k) => [p[0] + v[0] * k, p[1] + v[1] * k, p[2] + v[2] * k];
  const dir = (t) => [Math.sin(t) * radial[0], Math.sin(t) * radial[1], Math.cos(t)];
  const s2 = [config.base.x, config.base.y, L.shoulder_height];
  const s3 = add(s2, dir(a1), L.upper_arm);
  const s4 = add(s3, dir(a1 + a2), L.forearm);
  const direction = dir(a1 + a2 + a3);
  const flange = add(s4, direction, L.wrist);
  const grasp = add(flange, direction, gripperLength(config, angles[5]));
  // Pads separate along the radial direction (pads_axis x) or sideways (y) at zero roll, turned
  // about the gripper direction by -roll (positive roll is clockwise seen from above).
  const zero = (config.gripper.pads_axis || 'x') === 'y' ? [-Math.sin(yaw), Math.cos(yaw), 0] : radial;
  const padAxis = rotateAbout(zero, direction, -roll);
  return { s2, s3, s4, flange, grasp, direction, padAxis, gap: gapForLinkAngle(config, angles[5]) };
}

function rotateAbout(v, axis, angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  const dot = v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2];
  const cross = [axis[1] * v[2] - axis[2] * v[1], axis[2] * v[0] - axis[0] * v[2], axis[0] * v[1] - axis[1] * v[0]];
  return [0, 1, 2].map((i) => v[i] * c + cross[i] * s + axis[i] * dot * (1 - c));
}

// ---- minimal block physics --------------------------------------------------------------
// Blocks are upright boxes (half sizes hx, hy, hz about a centre). They fall under gravity,
// rest on the table or on a block whose footprint they overlap, get pushed by the pads, and
// are held when the pads close on them.

export class World {
  constructor(config) {
    this.config = config;
    this.gravity = 9.81;
    this.reset();
  }

  reset() {
    this.blocks = this.config.blocks.map((spec) => {
      const slot = this.config.slots[this.config.layout[spec.name]];
      const cube = spec.kind === 'cube';
      return {
        name: spec.name, kind: spec.kind, color: spec.color,
        hx: (cube ? spec.size : spec.width) / 2, hy: (cube ? spec.size : spec.depth) / 2, hz: (cube ? spec.size : spec.height) / 2,
        x: slot.x, y: slot.y, z: (cube ? spec.size : spec.height) / 2, yaw: slot.yaw || 0, vz: 0, held: false,
      };
    });
    this.held = null;
    this.events = [];
  }

  block(name) { return this.blocks.find((b) => b.name === name); }

  // Half extents of the block's footprint in table axes (upright box turned by yaw).
  footprint(b) {
    const c = Math.abs(Math.cos(b.yaw * DEG)), s = Math.abs(Math.sin(b.yaw * DEG));
    return { ex: b.hx * c + b.hy * s, ey: b.hx * s + b.hy * c };
  }

  overlaps(a, b, margin = 0) {
    const fa = this.footprint(a), fb = this.footprint(b);
    return Math.abs(a.x - b.x) < fa.ex + fb.ex - margin && Math.abs(a.y - b.y) < fa.ey + fb.ey - margin;
  }

  // Height of the surface under a block: table or the top of the highest overlapping block below.
  support(b) {
    let top = 0;
    for (const o of this.blocks) {
      if (o === b || o.held) continue;
      const oTop = o.z + o.hz;
      if (oTop <= b.z - b.hz + 0.002 && this.overlaps(b, o, 0.004)) top = Math.max(top, oTop);
    }
    return top;
  }

  // gripper: { left: [x,y,z], right: [x,y,z], gap, commandedGap, padHalf: [alongAxis, across, height] }
  step(dt, gripper) {
    const g = gripper;
    const axis = unit(sub(g.right, g.left));
    const centre = mid(g.left, g.right);
    const across = [-axis[1], axis[0], 0];
    if (this.held) {
      const b = this.held;
      if (g.commandedGap > 2 * Math.min(b.hx, b.hy) + 0.004) {
        this.held = b.held = false;
        b.vz = 0;
        this.events.push(`released ${b.name}`);
      } else {
        const p = add3(centre, mul(axis, b.offset[0]), mul(across, b.offset[1]));
        b.x = p[0]; b.y = p[1]; b.z = Math.max(centre[2] + b.offset[2], b.hz);
        b.yaw = b.offsetYaw + Math.atan2(axis[1], axis[0]) / DEG;
      }
    }
    for (const b of this.blocks) {
      if (b.held) continue;
      // Grasp: the pads are level with the block and close onto its faces.
      const rel = sub([b.x, b.y, b.z], centre);
      const along = Math.abs(dot(rel, axis)), side = Math.abs(dot(rel, across));
      const widthAlong = this.widthAlong(b, axis);
      const padsLevel = Math.abs(centre[2] - b.z) < b.hz + g.padHalf[2] - 0.002;
      if (!this.held && padsLevel && along < 0.006 && side < g.padHalf[1] + 0.004 && g.gap <= widthAlong + 0.003 && g.commandedGap < widthAlong) {
        this.held = b; b.held = true;
        b.offset = [dot(rel, axis), dot(rel, across), b.z - centre[2]];
        b.offsetYaw = b.yaw - Math.atan2(axis[1], axis[0]) / DEG;
        this.events.push(`holding ${b.name}`);
        continue;
      }
      // Push: a pad entering the block at its height shoves it sideways. Measured in the pad's
      // own frame: thin along the pad axis, wide across it.
      if (padsLevel) {
        const halfAlong = this.widthAlong(b, axis) / 2, halfAcross = this.widthAlong(b, across) / 2;
        for (const pad of [g.left, g.right]) {
          const d = sub([b.x, b.y, 0], [pad[0], pad[1], 0]);
          const dAlong = dot(d, axis), dAcross = dot(d, across);
          const penAlong = halfAlong + g.padHalf[0] - Math.abs(dAlong), penAcross = halfAcross + g.padHalf[1] - Math.abs(dAcross);
          if (penAlong > 0 && penAcross > 0) {
            const k = penAlong < penAcross ? mul(axis, Math.sign(dAlong || 1) * penAlong) : mul(across, Math.sign(dAcross || 1) * penAcross);
            b.x += k[0]; b.y += k[1];
            this.shove(b);
          }
        }
      }
      // Gravity and support.
      const floor = this.support(b) + b.hz;
      if (b.z > floor + 1e-4) {
        b.vz -= this.gravity * dt;
        b.z = Math.max(floor, b.z + b.vz * dt);
        if (b.z === floor) b.vz = 0;
      } else { b.z = floor; b.vz = 0; }
    }
    return this.events.splice(0);
  }

  widthAlong(b, axis) {
    const ang = Math.atan2(axis[1], axis[0]) - b.yaw * DEG;
    return 2 * (b.hx * Math.abs(Math.cos(ang)) + b.hy * Math.abs(Math.sin(ang)));
  }

  // A pushed block moves its neighbours out of the way instead of overlapping them.
  shove(b, depth = 0) {
    if (depth > 4) return;
    for (const o of this.blocks) {
      if (o === b || o.held || Math.abs(o.z - b.z) > b.hz + o.hz - 0.002) continue;
      const fb = this.footprint(b), fo = this.footprint(o);
      const dx = o.x - b.x, dy = o.y - b.y;
      const penX = fb.ex + fo.ex - Math.abs(dx), penY = fb.ey + fo.ey - Math.abs(dy);
      if (penX > 0 && penY > 0) {
        if (penX < penY) o.x += Math.sign(dx || 1) * penX; else o.y += Math.sign(dy || 1) * penY;
        this.shove(o, depth + 1);
      }
    }
  }
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const add3 = (a, b, c) => [a[0] + b[0] + c[0], a[1] + b[1] + c[1], a[2] + b[2] + c[2]];
const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a) => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };

// Pad positions for the physics from a forward-kinematics result.
export function gripperState(config, fk, commandedLinkAngle) {
  const half = fk.gap / 2;
  const g = config.gripper;
  return {
    left: [fk.grasp[0] - fk.padAxis[0] * half, fk.grasp[1] - fk.padAxis[1] * half, fk.grasp[2] - fk.padAxis[2] * half],
    right: [fk.grasp[0] + fk.padAxis[0] * half, fk.grasp[1] + fk.padAxis[1] * half, fk.grasp[2] + fk.padAxis[2] * half],
    gap: fk.gap, commandedGap: gapForLinkAngle(config, commandedLinkAngle),
    padHalf: [g.pad_thickness / 2, 0.008, g.pad_height / 2],
  };
}
