import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ARM_TAGS } from '../js/config.js';
import { fk } from '../js/kinematics.js';
import { calFromLookAt, tagPoints, projectBoard } from '../js/armtrack.js';
import { squarePose, fitPartPose, learnTagLayout } from '../js/rigidtrack.js';

const cams = [calFromLookAt([430, 160, 430], [110, 0, 120], 950), calFromLookAt([440, -200, 380], [110, 0, 120], 900)];
const angles = { s1: 8, s2: -15, s3: 80, s4: 70, s5: 30 };
const frames = fk(angles);
// what a camera reports for a tag: its corners, turned by `turn` quarter turns (a tag stuck on rotated)
const seen = (cal, tag, turn = 0, f = frames) => {
  const c = tagPoints(tag, null, f).corners.map(p => projectBoard(cal, p));
  return [0, 1, 2, 3].map(i => c[(i - turn + 4) % 4]);
};
const angleBetween = (A, B) => { let tr = 0; for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) tr += A[3 * i + k] * B[3 * i + k]; return Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI; };

test('squarePose: one tag in one camera gives its pose', () => {
  const tag = ARM_TAGS.find(t => t.id === 109);
  const sp = squarePose(cams[1], seen(cams[1], tag), tag.size);
  const truth = tagPoints(tag, null, frames).centre;
  assert.ok(Math.hypot(...sp.p.map((v, i) => v - truth[i])) < 0.01);
});

test('fitPartPose: each part from its tags, from one camera or two, with a tag stuck on turned', () => {
  for (const link of ['wrist', 'hand']) {
    const tags = ARM_TAGS.filter(t => t.link === link);
    const turn = { [tags[1].id]: 1 };                                   // second tag turned a quarter
    for (const views of [[0], [1], [0, 1]]) {
      const observations = views.map(ci => ({ cal: cams[ci], tags: Object.fromEntries(tags.map(t => [t.id, { corners: seen(cams[ci], t, turn[t.id] || 0) }])) }));
      const fit = fitPartPose(link, observations);
      assert.ok(fit.rms < 0.05, `${link} ${views}: rms ${fit.rms}`);
      assert.ok(Math.hypot(...fit.p.map((v, i) => v - frames[link].p[i])) < 0.01, `${link} ${views}: position`);
      assert.ok(angleBetween(fit.R, frames[link].R) < 0.05, `${link} ${views}: rotation`);
      assert.equal(fit.shifts[tags[1].id], 1);
    }
  }
});

test('learnTagLayout: measures where a tag really sits on its part', () => {
  const ref = ARM_TAGS.find(t => t.id === 109), other = ARM_TAGS.find(t => t.id === 108);
  const real = { ...other, centre: other.centre.map((v, i) => v + [0.3, -0.4, 0.2][i]) };   // stuck 5 mm off
  const observations = cams.map(cal => ({ cal, tags: { 109: { corners: seen(cal, ref) }, 108: { corners: seen(cal, real) } } }));
  // the reference alone (one square tag needs a starting guess for its quarter turn: the commanded pose, here 10 degrees off)
  const init = fk({ s1: 12, s2: -10, s3: 84, s4: 66, s5: 30 }).wrist;
  const pose = fitPartPose('wrist', observations.map(o => ({ cal: o.cal, tags: { 109: o.tags[109] } })), { init });
  const layout = learnTagLayout('wrist', observations, pose);
  assert.ok(Math.hypot(...layout[108].centre.map((v, i) => v - real.centre[i])) < 0.02, JSON.stringify(layout[108]));
});
