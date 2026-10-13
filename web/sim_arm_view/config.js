// Numbers of the arm simulation viewer. Lengths in cm, angles in degrees, servo values in
// PCA9685 counts at 60 Hz. Frame: origin on the board top under the S1 axis, Z up, +Y from the
// back edge toward the front, +X to the right seen from the robot.
//
// Arm, board, cube, gripper and servo calibration come from the simulation's API
// (GET /api/sim/model; src/roboter_arm/sim/domain/arm_model.json), the only copy. The rest is
// for drawing and the camera tools only.

const model = await (await fetch('/api/sim/model', { cache: 'no-store' })).json();

export const ARM = model.arm;
export const BOARD = model.board;
export const CUBE = model.cube;
export const GRIPPER = model.gripper;
// kinematics.js names: home = calibration reference count, qHome = model angle there, dir = sign,
// cpd = counts per degree.
export const SERVOS = model.servos.map(s => ({ ...s, home: s.ref, qHome: s.q_ref ?? 0, dir: s.direction ?? 1, cpd: s.span ? s.span[0] / s.span[1] : null }));

// Printed ArUco mat (config/camera-table-xy). Camera table frame: origin at the middle of the
// mat edge nearest the arm, +X away from the arm, +Y to the arm's left, mm.
// Board frame: x = -Y/10, y = X/10 + d.
export const MAT = {
  w: 28, d: 36,          // cm
  // d: S1 axis to the mat's near edge, cm. Read from an overhead photo: the board's back edge
  // lies at about mat X = 10 mm, and S1 is 1,5 in front of that edge. Not measured.
  sToEdge: -2.5,
};

// OAK camera, fitted from the mat markers.
export const CAMERA = {
  posCamMm: [480.8, -8.3, 371.7], // camera table frame
  fPx: 952, w: 1280, h: 720,
};

export const COLOURS = {
  orange: 0xe8792b, blue: 0x2f6fd6, yellow: 0xf2c230, green: 0x3aa356, red: 0xd23b3b,
};
