// three.js model of the arm, board, mat, cubes and the desk around them. Units: cm, Z up,
// board frame of docs/arm-dimensions.md.
//
//   textures.js   procedural canvas textures (mat, wood, labels, noise)
//   materials.js  physical materials
//   geometry.js   mesh helpers, plates with holes, merging static meshes (bake)
//   parts.js      kit parts: screws, MG996R servo, horns, bracket plates, zip ties
//   gripper.js    the claw and its linkage
//   cables.js     routed servo cables, ribbon and power leads
//   arm.js        the arm's joint groups and setPose
//   desk.js       board, mat, desk, Pi, power supply, sensor, OAK camera, cubes

export { makeMaterials } from './materials.js';
export { buildArm, BASE_OFFSET_Y } from './arm.js';
export { buildTable, buildCamera, buildCube } from './desk.js';
