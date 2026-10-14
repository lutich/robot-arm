// Every number the simulator uses, in one place.
// Lengths in cm, angles in degrees, servo values in PCA9685 counts at 60 Hz.
// Frame (docs/arm-dimensions.md): origin on the board top under the S1 axis, Z up,
// +Y from the back edge toward the front, +X to the right seen from the robot.
//
// The schematic (docs/images/arm-dimensions.png) is canon: link lengths, gripper
// values, board and cube come from it unchanged. Sizes of servo bodies and brackets
// are only for drawing and collision, taken from the photos and the MG996R datasheet.

export const ARM = {
  H_S2: 9.6,    // board top to S2 axis
  L2: 10.5,     // S2 shoulder to S3 elbow
  L3: 12.6,     // S3 elbow to S4 wrist flexion
  L4: 6.7,      // S4 wrist flexion to gripper flange (S5)
  G_BLOCK: 11.0 // flange to the centre of a held cube
};

export const BOARD = { w: 30, d: 38, t: 1.0, axisToBackEdge: 1.5 };

// Two 90 x 42 x 25 mm U-beams form the raised base fixture. Each beam has a
// lower and upper horizontal plate with an open cavity between them.
export const BASE = {
  w: 9.0, beamD: 4.0, h: 2.5, plateT: 0.2,
  offsetY: 2.2, beamY: 2.12, webY: 0.13, webT: 0.2,
};

export const CUBE = 2.65;

// Gripper parallel linkage, gripper-local u (sideways) and v (down from the flange), cm.
// Fitted in docs/arm-dimensions.md so that it reproduces the measured values exactly:
// open 10,0 / gap 4,5; holding the cube 11,0 / gap 2,65; closed 11,5 / gap 0.
export const GRIPPER = {
  gear: [0.85, 1.4],       // gear centre (u, v)
  outer: [2.2, 1.9],       // outer link pivot (u, v)
  link: 3.5,               // both links
  tip: [-1.515, 6.664],    // pad contact point relative to the gear-link end
  pad: { h: 1.2, t: 0.27, w: 1.0 },
  plate: { w: 6.4, h: 2.4, t: 0.4 },
  thetaOpen: 56.4,         // link angle from vertical at S6 = 490
  thetaClosed: 11.0,       // at S6 = 650
};

// Servo table. q_home is the model angle (0 = all links straight up,
// positive S2..S4 tilt toward +Y, positive S1 turns +Y toward +X) at the HOME count.
// dir: +1 if increasing counts increases the model angle.
//
// q_home, dir and cpd are NOT fully measured (HANDOVER.md, open item 1). HOME retains
// the photo-fitted angles. S2-S4 counts/degree are additionally anchored to the observed
// PARK pose, where the closed claw rests on the upper plate of the 2,5 cm raised fixture.
// The directions match the controller's historical sign table (S2 -1, S3 +1, S4 -1).
// Change them in the Calibration panel once they are measured.
export const SERVOS = [
  { ch: 0, name: 'S1', joint: 'Base rotation',  home: 320, park: 330, low: 90,  high: 660, cpd: 260 / 85, qHome: 0,  dir: -1 },
  { ch: 1, name: 'S2', joint: 'Shoulder',       home: 427, park: 540, low: 185, high: 660, cpd: 2.512007, qHome: 6,  dir: -1 },
  { ch: 2, name: 'S3', joint: 'Elbow',          home: 403, park: 580, low: 90,  high: 660, cpd: 3.525795, qHome: 62, dir: +1 },
  { ch: 3, name: 'S4', joint: 'Wrist flexion',  home: 220, park: 195, low: 90,  high: 660, cpd: 1.097333, qHome: 84, dir: -1 },
  // The live controller's HOME has S5 = 250 (docs/controller-live-2026-10-12.md).
  // At 480 (the documented HOME) the jaws close along the arm's sideways axis.
  { ch: 4, name: 'S5', joint: 'Wrist rotation', home: 250, park: 480, low: 90,  high: 660, cpd: 230 / 90, qHome: 90, dir: -1 },
  { ch: 5, name: 'S6', joint: 'Gripper',        home: 550, park: 650, low: 490, high: 660, cpd: null, open: 490, grip: 590, closed: 650 },
];

// Controller behaviour copied from lutich/robot-arm (control/domain/motion.py, docs/api.md).
export const CONTROLLER = {
  manualSpeed: 20,      // counts/s for move, jog, go_pose
  demoSpeed: 20,        // default demo playback speed, 1..100 counts/s
  demoSpeedMax: 100,
  stepSize: 10,         // jog step
  // Trajectory: cubic smoothstep, all joints share one duration, which is the
  // slowest joint's 1,5 x |delta| / speed (the smoothstep peaks at 1,5x average).
  peakFactor: 1.5,
};

// Printed ArUco mat (lutich/robot-arm config/camera-table-xy). Camera table frame:
// origin at the middle of the mat edge nearest the arm, +X away from the arm, +Y to the
// arm's left, mm. Board frame: x = -Y/10, y = X/10 + d.
export const MAT = {
  w: 28, d: 36,          // cm
  // d: S1 axis to the mat's near edge, cm. Read from the overhead photo
  // docs/images/photos/setup-top-view-base.jpg: the board's back edge lies at about mat
  // X = 10 mm (the paper overhangs it), and S1 is 1,5 in front of that edge, so S1 is at
  // about X = 25 mm. Not measured; HANDOVER.md open item 3.
  sToEdge: -2.5,
};

// OAK camera, fitted from the mat markers (docs/tracking-plan.md, section 2).
export const CAMERA = {
  posCamMm: [480.8, -8.3, 371.7], // camera table frame
  fPx: 952, w: 1280, h: 720,
};

// Motion planning defaults (docs/pick-and-place-plan.md, docs/tracking-plan.md).
export const PLAN = {
  hover: 5.0,           // approach height above a cube on the mat
  hoverStacked: 2.0,    // above level 1 and higher
  descentStep: 1.0,     // cm between IK waypoints on a vertical descent
  clearance: 0.05,      // collision tolerance, cm (0,5 mm)
  transitMargin: 0.15,  // planned paths keep 1,5 mm between the arm (or a held cube) and other cubes
};

export const COLOURS = {
  orange: 0xe8792b, blue: 0x2f6fd6, yellow: 0xf2c230, green: 0x3aa356, red: 0xd23b3b,
};
