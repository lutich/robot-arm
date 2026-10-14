import test from 'node:test';
import assert from 'node:assert/strict';
import { DICT, detectMarkers } from '../js/aruco.js';
import { GRIPPER_MARKER_ID, MARKER_LOCAL_CORNERS, estimateGripperMarkerPose } from '../js/gripper-tracker.js';

function renderMarker(rows, cell = 36, margin = 36) {
  const width = cell * 6 + margin * 2;
  const height = width;
  const data = new Uint8ClampedArray(width * height * 4).fill(255);
  for (let y = margin; y < margin + cell * 6; y += 1) for (let x = margin; x < margin + cell * 6; x += 1) {
    const row = Math.floor((y - margin) / cell), column = Math.floor((x - margin) / cell);
    const white = row > 0 && row < 5 && column > 0 && column < 5 && rows[row - 1][column - 1] === '1';
    if (!white) {
      const offset = 4 * (y * width + x);
      data[offset] = data[offset + 1] = data[offset + 2] = 0;
    }
  }
  for (let index = 3; index < data.length; index += 4) data[index] = 255;
  return { width, height, data };
}

test('dedicated DICT_4X4_50 marker id 6 is detected separately from mat ids 0 through 5', () => {
  assert.equal(GRIPPER_MARKER_ID, 6);
  assert.deepEqual(DICT[6], ['1001', '1110', '0010', '1110']);
  const { markers } = detectMarkers(renderMarker(DICT[6]), { window: 51 });
  assert.ok(markers[6], `detected ids: ${Object.keys(markers).join(', ') || 'none'}`);
});

test('square marker pose recovers its table position and calibrated TCP offset', () => {
  const calibration = {
    R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    t: [0, 0, 500],
    f: 800,
    cx: 640,
    cy: 360,
  };
  const markerTable = [50, 20, 100];
  const imageCorners = MARKER_LOCAL_CORNERS(30).map(point => {
    const camera = point.map((value, axis) => value + markerTable[axis] + calibration.t[axis]);
    return [calibration.f * camera[0] / camera[2] + calibration.cx, calibration.f * camera[1] / camera[2] + calibration.cy];
  });
  const result = estimateGripperMarkerPose(calibration, imageCorners, { markerToTcpMm: [10, -5, 20], sizeMm: 30, timestamp: 123 });
  result.markerTableMm.forEach((value, axis) => assert.ok(Math.abs(value - markerTable[axis]) < 1e-6));
  [60, 15, 120].forEach((value, axis) => assert.ok(Math.abs(result.tcpTableMm[axis] - value) < 1e-6));
  assert.ok(result.reprojectionRmsPx < 1e-8);
  assert.equal(result.timestamp, 123);
});

test('marker tracking rejects uncalibrated dimensions and malformed corners', () => {
  const calibration = { R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], t: [0, 0, 500], f: 800, cx: 640, cy: 360 };
  assert.throws(() => estimateGripperMarkerPose(calibration, [], { sizeMm: 30 }), /four finite image corners/);
  assert.throws(() => estimateGripperMarkerPose(calibration, [[0, 0], [1, 0], [1, 1], [0, 1]], { sizeMm: 0 }), /size must be positive/);
});
