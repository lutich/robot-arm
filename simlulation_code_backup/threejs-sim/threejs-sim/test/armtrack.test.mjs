import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CAMERAS, ARM_TAGS as ALL_TAGS } from '../js/config.js';
const ARM_TAGS = ALL_TAGS.filter(t => t.id != null);
import { calFromLookAt, tagPoints, projectBoard, triangulate, fitArmPose, boardToTable } from '../js/armtrack.js';
import { project } from '../js/calib.js';

const cams = CAMERAS.map(c => calFromLookAt(c.posMm, c.lookAtMm, c.fPx, c.w, c.h));
const truth = { s1: 12, s2: -20, s3: 95, s4: 70, s5: 25 };

test('calFromLookAt: the target projects to the image centre, the camera centre is right', () => {
  for (const [i, c] of CAMERAS.entries()) {
    const [u, v] = project(cams[i], c.lookAtMm);
    assert.ok(Math.abs(u - c.w / 2) < 1e-6 && Math.abs(v - c.h / 2) < 1e-6);
    const C = [0, 1, 2].map(k => -(cams[i].R[0][k] * cams[i].t[0] + cams[i].R[1][k] * cams[i].t[1] + cams[i].R[2][k] * cams[i].t[2]));
    C.forEach((x, k) => assert.ok(Math.abs(x - c.posMm[k]) < 1e-6));
  }
});

test('triangulate: both cameras see a tag -> its 3D position to well under a millimetre', () => {
  for (const tag of ARM_TAGS) {
    const P = tagPoints(tag, truth).centre;
    const r = triangulate(cams.map(cal => ({ cal, uv: projectBoard(cal, P) })));
    const T = boardToTable(P);
    assert.ok(Math.hypot(...r.X.map((x, i) => x - T[i])) < 0.01, `tag ${tag.id}`);
    assert.ok(r.rms < 1e-3);
  }
  // with 0,5 px of noise the error stays in the millimetre range
  const P = tagPoints(ARM_TAGS[0], truth).centre, T = boardToTable(P);
  const r = triangulate(cams.map((cal, i) => { const q = projectBoard(cal, P); return { cal, uv: [q[0] + (i ? 0.5 : -0.5), q[1] + 0.5] }; }));
  assert.ok(Math.hypot(...r.X.map((x, i) => x - T[i])) < 3);
});

test('fitArmPose: joint angles from the two tags in two cameras', () => {
  const observations = cams.map(cal => ({ cal, tags: Object.fromEntries(ARM_TAGS.map(t => {
    const tp = tagPoints(t, truth);
    return [t.id, { centre: projectBoard(cal, tp.centre), corners: tp.corners.map(c => projectBoard(cal, c)) }];
  })) }));
  const init = { s1: 0, s2: -33, s3: 110, s4: 107, s5: 0 };            // e.g. the fitted PARK
  for (const useCorners of [false, true]) {
    const fit = fitArmPose(observations, { init, useCorners, prior: 1e6 });
    assert.ok(fit.rms < 0.05, `rms ${fit.rms}`);
    for (const j of ['s1', 's2', 's3', 's4', 's5']) assert.ok(Math.abs(fit.angles[j] - truth[j]) < 0.2, `${j}: ${fit.angles[j]} (corners ${useCorners})`);
  }
});
