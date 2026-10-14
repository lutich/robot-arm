// node --test test/*.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ARM, ARM_TAGS, MAT } from '../js/config.js';
import { fk, point } from '../js/kinematics.js';
import { calFromLookAt, tagPoints, projectBoard, boardToTable } from '../js/armtrack.js';
import { recalibrate, targetPoint } from '../js/recalib.js';

// Synthetic captures from a "real" arm whose wrist tags sit 0,6 cm higher and whose grip is 0,8 cm shorter than the
// model says: the recalibration must find both and bring the grip error down.
test('recalibration recovers a wrist-tag shift and the grip length from captures with targets', () => {
  const cams = [calFromLookAt([480.8, -8.3, 371.7], [175, 0, 0], 952), calFromLookAt([600, -320, 420], [170, 0, 0], 900)];
  cams.forEach(c => { c.rms = 0.5; });
  const truth = { wristZ: -0.6, grip: ARM.G_BLOCK - 0.8 };
  const poses = [
    { s1: -30, s2: 15, s3: 70, s4: 95, s5: 10 }, { s1: 25, s2: 5, s3: 85, s4: 90, s5: -40 }, { s1: 0, s2: 30, s3: 55, s4: 95, s5: 60 },
    { s1: -10, s2: -5, s3: 95, s4: 85, s5: -80 }, { s1: 35, s2: 20, s3: 60, s4: 100, s5: 0 }, { s1: -40, s2: 0, s3: 90, s4: 88, s5: 30 },
  ];
  const real = ARM_TAGS.filter(t => t.id != null).map(t => ({ ...t, centre: t.link === 'wrist' ? [t.centre[0], t.centre[1], t.centre[2] + truth.wristZ] : t.centre }));
  const captures = poses.map((a, i) => {
    const f = fk(a), grip = boardToTable(point(f.hand, [0, 0, truth.grip]));
    const views = cams.map(cal => ({ cal, tags: Object.fromEntries(real.map(t => [t.id, { corners: tagPoints(t, a, f).corners.map(P => projectBoard(cal, P)) }])) }));
    return { name: `pose ${i}`, views, target: { at: grip } };
  });
  const r = recalibrate(captures, { fit: ['wristZ', 'grip'] });
  assert.ok(Math.abs(r.change.wristZ - truth.wristZ) < 0.05, `wristZ ${r.change.wristZ}`);
  assert.ok(Math.abs(r.values.grip - truth.grip) < 0.05, `grip ${r.values.grip}`);
  for (const a of r.after) assert.ok(a.gripMm < 1, `${a.name}: grip error ${a.gripMm} mm after`);
  assert.ok(r.before.some(b => b.gripMm > 5), 'the model before was visibly off');
  // the model itself is untouched (applyRecalibration is a separate step)
  assert.equal(MAT.sToEdge, -2.5);
  assert.deepEqual(targetPoint({ at: [160, -110], level: 1 }), [160, -110, 39.75]);
});
