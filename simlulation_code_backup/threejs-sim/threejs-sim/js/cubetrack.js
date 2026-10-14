// Live cube tracking: the cubes in 3D from two or more cameras (locate3d.js) at their real height, on the mat,
// on other cubes, in the gripper or in the air; nothing is projected onto the mat. With the tracked arm pose,
// coloured blobs that are really the arm (its orange and red servo cables, the pink foam, the red Pi case
// behind it) are left out: a "cube" whose centre lies on the arm's collision shapes is the arm. A cube between
// the fingers, near the grip point, is the held cube.
// Pure (no page): runs in the page for every live round.

import { CUBE, BOARD, MAT } from './config.js';
import { fk } from './kinematics.js';
import { armShapes, segBoxDist, segSegDist } from './collision.js';
import { locateCubes, sizeCheck, plausibleSize } from './locate3d.js';

const H = CUBE / 2;   // half a cube, cm

/** Distance (cm) from a point to a collision shape's surface; negative inside. */
function pointShapeDist(P, s) {
  if (s.kind === 'box') return segBoxDist(P, P, s);
  if (s.kind === 'capsule') return segSegDist(P, P, s.a, s.b) - s.r;
  return Infinity;
}

/**
 * views: [{ cal, detections }] (detect.js per camera); arm: { angles, gap } of the tracked arm, or null.
 * Returns [{ colour, board: [x, y, z] cm (measured cube centre), shown (resting cubes on their level), X (table mm),
 * yaw, level, where, gap, cameras }],
 * where = 'mat' | 'desk' | 'stack' (resting on a stack level) | 'gripper' | 'air'; blobs on the arm are dropped
 * (returned separately as `onArm` for the overlay's counts).
 */
export function trackCubes(views, arm = null) {
  const found = locateCubes(views);
  let shapes = [], grasp = null;
  if (arm?.angles) {
    const f = fk(arm.angles);
    grasp = f.grasp;
    shapes = armShapes(f, arm.gap ?? 3).filter(s => !/finger|pad|link/i.test(s.name));   // the jaws hold the cube
  }
  const cubes = [], onArm = [];
  for (const c of found) {
    const P = c.board;
    const held = grasp && Math.hypot(P[0] - grasp[0], P[1] - grasp[1], P[2] - grasp[2]) < 2.0;
    if (!held && shapes.some(s => pointShapeDist(P, s) < 0.8)) { onArm.push(c); continue; }
    if (c.X[0] < -30) { onArm.push(c); continue; }        // behind the mat's near edge: the red Pi case and cables, not a cube
    if (P[2] < -BOARD.t - 0.5) { onArm.push(c); continue; }   // below the desk: two blobs paired wrongly
    // the known cube size: a blob far smaller or larger than a cube there, or of another shape, is something else
    const fit = sizeCheck(c);
    if (c.seen?.length && !plausibleSize(fit)) { onArm.push({ ...c, size: fit }); continue; }   // (cubes added for a merged blob have no blob of their own)
    // off the mat a cube stands on the desk, the board's thickness lower
    // a cube still stands on the mat while its centre is less than half a cube beyond the edge
    const m = H * 10 + 3, onMat = c.X[0] > -m && c.X[0] < MAT.d * 10 + m && Math.abs(c.X[1]) < MAT.w * 5 + m;
    const base = onMat ? 0 : -BOARD.t;
    const level = Math.max(0, Math.round((P[2] - base - H) / CUBE));
    const resting = Math.abs(P[2] - (base + H + level * CUBE)) < 0.9;
    // resting cubes are drawn exactly on their level (the measured height is a few mm off); held or flying ones as measured
    const shown = resting && !held ? [P[0], P[1], base + H + level * CUBE] : P.slice();
    cubes.push({ ...c, level, shown, where: held ? 'gripper' : !resting ? 'air' : level ? 'stack' : onMat ? 'mat' : 'desk' });
  }
  cubes.onArm = onArm;
  return cubes;
}

/** Short text for a tracked cube: "blue · 1,3 cm · mat", "orange · 4,0 cm · level 1", "red · 6,2 cm · in the gripper". */
export function cubeText(c) {
  const z = (c.shown || c.board)[2], h = `${(Math.round(z * 2) / 2).toFixed(1).replace('.', ',')} cm`;   // 0,5 cm steps: the label changes less often
  const where = { mat: 'on the mat', desk: 'on the desk', stack: `level ${c.level}`, gripper: 'in the gripper', air: 'in the air' }[c.where];
  return `${c.colour} · ${h} · ${where}`;
}
