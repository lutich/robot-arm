import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ARM_TAGS } from '../js/config.js';
import { fk } from '../js/kinematics.js';
import { calFromLookAt, tagPoints, projectBoard } from '../js/armtrack.js';
import { fitPartPose } from '../js/rigidtrack.js';
import { fitJointPair, estimateJointAxis } from '../js/jointfit.js';

const cams = [calFromLookAt([430, 160, 430], [110, 0, 120], 950), calFromLookAt([440, -200, 380], [110, 0, 120], 900)];
const IDS = ARM_TAGS.filter(t => t.id != null);
// what the cameras see of tags placed as `tags` (the real arm), at joint angles a
const observe = (tags, a) => { const f = fk(a); return cams.map(cal => ({ cal, tags: Object.fromEntries(tags.map(t => [t.id, { corners: tagPoints(t, null, f).corners.map(p => projectBoard(cal, p)) }])) })); };
const base = { s1: 6, s2: -12, s3: 75, s4: 70 };

test('fitJointPair: wrist pose and S5 from the tags of both parts', () => {
  for (const s5 of [-40, 0, 35, 80]) {
    const fit = fitJointPair('s5', observe(IDS, { ...base, s5 }), { init: { parent: fk({ ...base, s5: 0 }).wrist } });
    assert.ok(fit.rms < 0.05, `rms ${fit.rms}`);
    assert.ok(Math.abs(fit.angle - s5) < 0.05, `s5 ${s5}: ${fit.angle}`);
  }
});

test('estimateJointAxis: recovers the gripper tags stuck on elsewhere, and the S5 offset', () => {
  const realHand = IDS.map(t => (t.link === 'hand' ? { ...t, centre: t.centre.map((v, i) => v + [0.8, -0.5, 0.6][i]) } : t));
  const records = [];
  for (const [s5, s4] of [[-30, 70], [10, 60], [50, 75], [85, 65]]) {
    const a = { ...base, s4, s5 }, f = fk(a), obs = observe(realHand, a);
    const parent = fitPartPose('wrist', obs, { init: f.wrist }), child = fitPartPose('hand', obs, { init: f.hand });
    records.push({ parent, child, commanded: s5 - 7 });        // the servo's HOME angle is 7 degrees off
  }
  const r = estimateJointAxis('s5', records);
  assert.ok(r.model.axisDeg < 0.1 && r.model.offset < 0.02, JSON.stringify(r.model));
  assert.ok(r.fit < 0.01);
  for (const t of realHand.filter(x => x.link === 'hand')) {
    const got = r.layout.find(x => x.key === t.key);
    assert.ok(Math.hypot(...got.centre.map((v, i) => v - t.centre[i])) < 0.02, `${t.id}: ${got.centre} vs ${t.centre}`);
  }
  assert.ok(Math.abs(r.servo.slope - 1) < 0.01 && Math.abs(r.servo.offset - 7) < 0.1, JSON.stringify(r.servo));
});
