export const RECORDED_POSE_CALIBRATION = Object.freeze([
  { name: 'Pos1_left', matMm: [160, -110], counts: [235, 347, 360, 185, 380, 590], provenance: 'measured' },
  { name: 'Pos1_right', matMm: [160, -110], counts: [440, 347, 360, 185, 340, 590], provenance: 'user-confirmed duplicate' },
  { name: 'Pos2_left', matMm: [120, -125], counts: [200, 417, 470, 225, 98, 590], provenance: 'measured' },
  { name: 'Pos2_right', matMm: [115, 120], counts: [475, 417, 470, 225, 622, 590], provenance: 'measured' },
  { name: 'Pos3_left', matMm: [50, -130], counts: [130, 457, 530, 255, 278, 590], provenance: 'measured' },
  { name: 'Pos3_right', matMm: [50, 130], counts: [505, 457, 530, 255, 652, 590], provenance: 'inferred/provisional' },
]);

export function markerCentres(markerCorners) {
  return Object.entries(markerCorners).map(([id, corners]) => ({
    id: Number(id),
    matMm: [
      corners.reduce((sum, corner) => sum + corner[0], 0) / corners.length,
      corners.reduce((sum, corner) => sum + corner[1], 0) / corners.length,
    ],
  }));
}

const distance2d = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

export function calibrationRows(markerCorners, observation = {}) {
  const centres = markerCentres(markerCorners);
  const observedMarkers = new Map((observation.markers ?? []).map(marker => [marker.id, marker]));
  const cubes = observation.cubes ?? [];

  return RECORDED_POSE_CALIBRATION.map(pose => {
    const marker = centres.reduce((nearest, candidate) => (
      !nearest || distance2d(pose.matMm, candidate.matMm) < distance2d(pose.matMm, nearest.matMm)
        ? candidate
        : nearest
    ), null);
    const cube = cubes.reduce((nearest, candidate) => (
      !nearest || distance2d(pose.matMm, candidate.tableMm) < distance2d(pose.matMm, nearest.tableMm)
        ? candidate
        : nearest
    ), null);
    const cubeDeltaMm = cube ? distance2d(pose.matMm, cube.tableMm) : Infinity;
    const toleranceMm = pose.matMm[0] < 100 ? 10 : 2;

    return {
      ...pose,
      toleranceMm,
      marker: {
        ...marker,
        deltaMm: distance2d(pose.matMm, marker.matMm),
        cameraPixel: observedMarkers.get(marker.id)?.pixel ?? null,
      },
      cameraCube: cube && cubeDeltaMm <= 35 ? { ...cube, deltaMm: cubeDeltaMm } : null,
    };
  });
}
