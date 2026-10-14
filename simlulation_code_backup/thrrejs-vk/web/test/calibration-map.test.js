import test from 'node:test';
import assert from 'node:assert/strict';
import { MARKER_CORNERS } from '../js/calib.js';
import { RECORDED_POSE_CALIBRATION, calibrationRows, markerCentres } from '../js/calibration-map.js';

test('recorded grab poses preserve the observed coordinates and servo counts', () => {
  assert.deepEqual(RECORDED_POSE_CALIBRATION.map(({ name, matMm, counts }) => ({ name, matMm, counts })), [
    { name: 'Pos1_left', matMm: [160, -110], counts: [235, 347, 360, 185, 380, 590] },
    { name: 'Pos1_right', matMm: [160, -110], counts: [440, 347, 360, 185, 340, 590] },
    { name: 'Pos2_left', matMm: [120, -125], counts: [200, 417, 470, 225, 98, 590] },
    { name: 'Pos2_right', matMm: [115, 120], counts: [475, 417, 470, 225, 622, 590] },
    { name: 'Pos3_left', matMm: [50, -130], counts: [130, 457, 530, 255, 278, 590] },
    { name: 'Pos3_right', matMm: [50, 130], counts: [505, 457, 530, 255, 652, 590] },
  ]);
});

test('calibration rows connect poses to fixed markers and current camera pixels', () => {
  assert.deepEqual(markerCentres(MARKER_CORNERS).map(marker => marker.matMm), [
    [25, 110], [25, -110], [160, 110], [160, -110], [335, 110], [335, -110],
  ]);
  const rows = calibrationRows(MARKER_CORNERS, {
    markers: [{ id: 3, pixel: [640.5, 420.25] }],
    cubes: [{ colour: 'orange', tableMm: [160, -110, 13.25], ok: true }],
  });
  assert.equal(rows[0].marker.id, 3);
  assert.deepEqual(rows[0].marker.cameraPixel, [640.5, 420.25]);
  assert.equal(rows[0].cameraCube.colour, 'orange');
  assert.equal(rows[4].toleranceMm, 10);
  assert.equal(rows[2].toleranceMm, 2);
  assert.equal(rows[5].provenance, 'inferred/provisional');
});
