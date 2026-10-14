import { homography } from './calib.js';

export const GRIPPER_MARKER_ID = 6;
export const DEFAULT_GRIPPER_MARKER = Object.freeze({
  dictionary: 'DICT_4X4_50',
  id: GRIPPER_MARKER_ID,
  sizeMm: 30,
  markerToTcpMm: [0, 0, 0],
});

export const MARKER_LOCAL_CORNERS = sizeMm => {
  const h = sizeMm / 2;
  // OpenCV corner order: canonical top-left, top-right, bottom-right, bottom-left.
  // Marker +X points right, +Y to the printed top and +Z out of the printed face.
  return [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]];
};

const dot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
const norm = vector => Math.hypot(...vector);
const scale = (vector, amount) => vector.map(value => value * amount);
const subtract = (a, b) => a.map((value, index) => value - b[index]);
const add = (a, b) => a.map((value, index) => value + b[index]);
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const normalize = vector => {
  const length = norm(vector);
  if (!(length > 1e-9)) throw new Error('Degenerate gripper-marker pose');
  return scale(vector, 1 / length);
};
const transpose = matrix => matrix[0].map((_, column) => matrix.map(row => row[column]));
const multiply = (matrix, vector) => matrix.map(row => dot(row, vector));
const multiplyMatrices = (a, b) => a.map(row => transpose(b).map(column => dot(row, column)));

function cameraRayColumn(calibration, column) {
  return [
    (column[0] - calibration.cx * column[2]) / calibration.f,
    (column[1] - calibration.cy * column[2]) / calibration.f,
    column[2],
  ];
}

function markerCameraPose(calibration, imageCorners, sizeMm) {
  const local = MARKER_LOCAL_CORNERS(sizeMm);
  const H = homography(local.map(([x, y]) => [x, y]), imageCorners);
  const columns = [0, 1, 2].map(column => H.map(row => row[column]));
  let b1 = cameraRayColumn(calibration, columns[0]);
  let b2 = cameraRayColumn(calibration, columns[1]);
  let t = cameraRayColumn(calibration, columns[2]);
  let amount = 2 / (norm(b1) + norm(b2));
  if (t[2] * amount < 0) amount *= -1;
  b1 = scale(b1, amount);
  b2 = scale(b2, amount);
  t = scale(t, amount);

  const r1 = normalize(b1);
  const r2 = normalize(subtract(b2, scale(r1, dot(r1, b2))));
  const r3 = normalize(cross(r1, r2));
  const rotation = [
    [r1[0], r2[0], r3[0]],
    [r1[1], r2[1], r3[1]],
    [r1[2], r2[2], r3[2]],
  ];
  return { rotation, translation: t, local };
}

function projectCameraPoint(calibration, point) {
  if (!(point[2] > 1e-6)) throw new Error('Gripper marker is behind the calibrated camera');
  return [
    calibration.f * point[0] / point[2] + calibration.cx,
    calibration.f * point[1] / point[2] + calibration.cy,
  ];
}

/**
 * Estimate the rigid gripper marker and TCP pose in the calibrated table frame.
 * `markerToTcpMm` is expressed in marker coordinates (+Z out of the printed face).
 */
export function estimateGripperMarkerPose(calibration, imageCorners, {
  sizeMm = DEFAULT_GRIPPER_MARKER.sizeMm,
  markerToTcpMm = DEFAULT_GRIPPER_MARKER.markerToTcpMm,
  timestamp = Date.now(),
} = {}) {
  if (!calibration?.R || !calibration?.t || !Number.isFinite(calibration.f)) throw new Error('Calibrate the camera before tracking the gripper marker');
  if (!Array.isArray(imageCorners) || imageCorners.length !== 4 || imageCorners.some(point => !Array.isArray(point) || point.length !== 2 || point.some(value => !Number.isFinite(value)))) {
    throw new Error('Gripper marker requires four finite image corners');
  }
  if (!(Number.isFinite(sizeMm) && sizeMm > 0)) throw new Error('Gripper marker size must be positive');
  if (!Array.isArray(markerToTcpMm) || markerToTcpMm.length !== 3 || markerToTcpMm.some(value => !Number.isFinite(value))) {
    throw new Error('Marker-to-TCP offset must contain three finite millimetre values');
  }

  const cameraPose = markerCameraPose(calibration, imageCorners, sizeMm);
  const cameraToTable = transpose(calibration.R);
  const markerTableMm = multiply(cameraToTable, subtract(cameraPose.translation, calibration.t));
  const rotationTable = multiplyMatrices(cameraToTable, cameraPose.rotation);
  const tcpTableMm = add(markerTableMm, multiply(rotationTable, markerToTcpMm));
  const normalTable = rotationTable.map(row => row[2]);

  let squaredError = 0;
  cameraPose.local.forEach((point, index) => {
    const cameraPoint = add(multiply(cameraPose.rotation, point), cameraPose.translation);
    const projected = projectCameraPoint(calibration, cameraPoint);
    squaredError += (projected[0] - imageCorners[index][0]) ** 2 + (projected[1] - imageCorners[index][1]) ** 2;
  });

  return {
    markerId: GRIPPER_MARKER_ID,
    timestamp,
    sizeMm,
    markerToTcpMm: [...markerToTcpMm],
    imageCorners: imageCorners.map(point => [...point]),
    markerTableMm,
    tcpTableMm,
    rotationTable,
    normalTable,
    reprojectionRmsPx: Math.sqrt(squaredError / imageCorners.length),
  };
}

export function distance3(a, b) {
  return Math.hypot(...a.map((value, index) => value - b[index]));
}
