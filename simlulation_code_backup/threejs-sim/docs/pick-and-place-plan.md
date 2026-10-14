# Plan: move a coloured block from a start position to a target position

Goal: the arm picks up the coloured test cube at a known start position on the board and puts it down at a known target position.

For now this runs **open loop, without a camera**.
The arm computes every servo value from the measured link lengths (`arm-dimensions.md`) and the measured ticks per degree (`servo-calibration.md`).
A camera may come later, only for fine adjustments near the block (see section 6).

All lengths in **cm**, all servo values in ticks.
Coordinates follow the Blender setup in `arm-dimensions.md`:
origin on the board top directly under the S1 axis, Z up, Y from the back edge toward the front, X along the 30 cm edge.

## 1. Assumptions

| Assumption | Why it matters |
|------------|----------------|
| Every run starts from the calibrated home position 320 / 427 / 403 / 220 / 480 / 650 | All tick values are computed as offsets from home |
| The joint angle of each servo at its home tick is known | Needed to turn a computed angle into ticks, see section 3 |
| Ticks per degree are linear over the whole range | We only measured one 90° move per joint |
| Start and target positions are given as board coordinates (x, y) of the cube centre | No camera, so the positions must be measured or placed on marks |
| The cube is the 2,65 cm test cube, lying flat, with its faces parallel to the board edges | Sets the grasp height and the S5 wrist rotation |
| The board is flat and the robot base does not move on it | Any shift of the base moves every position by the same amount |

## 2. Inputs

| Input | Example | Unit |
|-------|---------|------|
| Start position of the cube centre (x_s, y_s) | (5, 15) | cm |
| Target position of the cube centre (x_t, y_t) | (-8, 12) | cm |
| Cube size | 2,65 | cm |
| Hover height above the cube for the approach | 5 | cm |

Grasp height: the cube centre sits at z_c = 2,65 / 2 = 1,325 above the board.

## 3. From a board position to servo ticks

The arm grasps **from above**, with the gripper pointing straight down.
This keeps the kinematics simple: S2, S3 and S4 move in one vertical plane, S1 turns that plane toward the block.

### Step 1: base rotation (S1)

```
theta1 = atan2(x, y)          angle from the +Y axis, in degrees
r      = sqrt(x² + y²)        horizontal distance from the S1 axis
```

### Step 2: wrist point

With the gripper pointing down, the S4 wrist axis sits straight above the cube centre.
Tool length from S4 to the centre of a held cube: 6,7 + 11,0 = 17,7.

```
wrist height above the board  z_w = z_c + hover + 17,7
relative to the S2 shoulder   dz  = z_w - 9,6
                              d   = sqrt(r² + dz²)
```

### Step 3: shoulder and elbow (S2, S3)

Two-link inverse kinematics with L2 = 10,5 (S2 to S3) and L3 = 12,6 (S3 to S4).
Angles are measured in the arm plane, from the horizontal, positive upward.

```
cos(q3) = (d² - L2² - L3²) / (2 · L2 · L3)
q3      = -acos(...)                                   elbow relative angle, elbow-up solution
phi2    = atan2(dz, r) - atan2(L3·sin q3, L2 + L3·cos q3)    upper arm angle from horizontal
phi3    = phi2 + q3                                    forearm angle from horizontal
```

The elbow-up solution keeps the elbow above the block and away from the board.

### Step 4: wrist flexion (S4)

The tool must point down, so its absolute angle is -90°:

```
q4 = -90° - phi3
```

### Step 5: wrist rotation (S5)

S1 turns the gripper by theta1, so the jaws are no longer parallel to the cube faces.
S5 turns back by the same angle, folded into ±45° because a cube looks the same every 90°:

```
q5 = -theta1, wrapped into the range -45° to +45°
```

### Step 6: angles to ticks

```
ticks = home_tick + direction · (q - q_home) · ticks_per_degree
```

| Servo | Home tick | Ticks per degree | q_home (angle at home tick) | direction (+1 or -1) |
|-------|-----------|------------------|-----------------------------|----------------------|
| S1 | 320 | 3,06 | **open** | **open** |
| S2 | 427 | 2,44 | **open** | **open** |
| S3 | 403 | 2,67 | **open** | **open** |
| S4 | 220 | 2,78 | **open** | **open** |
| S5 | 480 | 2,56 | **open** | **open** |

Ticks are PCA9685 counts (see `reference-robot-arm-repo.md`).
Round to whole counts and check them against the controller's guards (S1, S3 to S5: 90 to 660; S2: 185 to 660).
The controller rejects targets outside its bounds instead of clamping them, so a pose that does not fit is unreachable and must not be sent.
The gripper (S6) does not use this formula, it switches between fixed values (section 4).

### Worked example

Cube at (5, 15), grasp height, no hover:

| Value | Result |
|-------|--------|
| theta1 | 18,4° |
| r | 15,81 |
| dz | 9,425 |
| phi2 (upper arm) | 72,1° |
| q3 (elbow) | -74,7° |
| phi3 (forearm) | -2,6° |
| q4 (wrist) | -87,4° |
| q5 (wrist rotation) | -18,4° |

A forward check with these angles puts the wrist at r = 15,81, z = 19,025, and the cube centre at z = 1,325, as intended.

The formulas are implemented in `threejs-sim/js/kinematics.js` (`ikTopDown`) and checked by `threejs-sim/test/core.test.mjs`, see section 8.

## 4. Motion sequence

Each step is one target pose.
Between poses the controller moves all joints in joint space at the demo speed set in the UI.

| # | Step | Pose | Gripper (S6) |
|---|------|------|--------------|
| 1 | Start | Home | 650 |
| 2 | Open gripper | Home | Open value |
| 3 | Move above start | Start position, hover 5 | Open |
| 4 | Descend | Start position, hover 0 | Open |
| 5 | Grip | Start position, hover 0 | 580 (block grip) |
| 6 | Lift | Start position, hover 5 | 580 |
| 7 | Move above target | Target position, hover 5 | 580 |
| 8 | Descend | Target position, hover 0 | 580 |
| 9 | Release | Target position, hover 0 | Open |
| 10 | Lift | Target position, hover 5 | Open |
| 11 | Return | Home | 650 |

"Open value" is 490: the team's taught demo on the controller opens at 490 and grips the cube at 590 (`controller-live-2026-10-12.md`).
That demo is also a hand-taught version of this sequence, but it lowers the gripper by moving only S2, so the gripper moves on an arc instead of straight down.

The UI already has a **Recorded Demo** panel ("Add position", "Run all once").
The computed poses can be entered there as a recorded demo to test the sequence before any code drives the arm directly.
Later, a script can send the poses through the controller's HTTP API (`move` per channel, or a demo file plus `demo/run`), see `reference-robot-arm-repo.md`.
Demo playback runs at 1 to 100 counts/s; start slow.

## 5. Workspace and accuracy

### Reach with a top-down grasp

The upper arm and forearm together reach at most 10,5 + 12,6 = 23,1 from the shoulder.

| Gripper height | dz | Maximum horizontal distance r from S1 |
|----------------|----|---------------------------------------|
| Grasp (hover 0) | 9,425 | 21,1 |
| Hover 5 | 14,425 | 18,0 |

The hover pose is the tighter limit, so **start and target positions should lie within about 17 cm of the S1 axis**.
That keeps a safety margin and avoids a fully stretched elbow, where small tick errors give large position errors.
The board reaches 36,5 cm in front of the S1 axis, so most of the board is out of reach for a top-down grasp.
Positions further out need an angled grasp, which is not part of this plan.

### Error budget without a camera

| Source | Effect |
|--------|--------|
| 1° error at S1 with r = 20 | 0,35 cm sideways |
| Ticks per degree from a single 90° move | A few percent per joint, adds up along the chain |
| Unknown home angles (q_home) | Constant offset in every pose until they are measured |
| Servo dead band and gear play | Small, but not measured |
| Link bending under the cube's weight | Not measured, probably small for a 2,65 cm cube |

Allowed error when gripping: the open gripper is 4,5 wide and the cube 2,65, so the cube may sit about 0,9 cm off centre and still be caught.
Height errors are more critical: if the gripper comes down too low it pushes against the board.
Start with a grasp height a few mm above the computed value and lower it in small steps.

## 6. Later: camera for fine adjustments

The camera is not used for the first version.
When it is added, it should only correct the last few millimetres, not replace the open-loop plan:

1. Move to the hover pose above the expected position, as in section 4.
2. Detect the coloured cube in the camera image and measure its offset from the expected position.
3. Correct x and y by that offset, recompute the ticks, and descend.

This needs a fixed camera position and a calibration from image pixels to board centimetres.
Later the camera could also find the start position on its own, so the start position no longer has to be measured by hand.

The controller repo already has most of this: an OAK camera and a fitted pixel-to-table mapping on a printed ArUco mat, with about 1 mm accuracy on the mat surface.
Its table frame differs from ours (origin at the mat edge, X forward, Y left, mm).
The conversion and the missing offset are in `reference-robot-arm-repo.md`, section 3.
The mapping is only valid on the mat surface; the top of a 2,65 cm cube appears shifted by perspective.

Update: `tracking-plan.md` replaces the stored mapping with a camera model fitted to the mat markers in every frame.
It finds cubes on the mat and on stacks, RGB only, and covers the stacking task.

## 7. Before the first real run

These have to be answered first, most of them are already listed in `servo-calibration.md` and `arm-dimensions.md`:

- **Home angles:** measure the joint angle of S1 to S5 in the home pose (q_home in section 3).
- **Directions:** for each servo, which tick direction is up, left or open.
- **S4 range:** from home, S4 can move only 130 counts (about 47°) in the minus direction before the 90 guard. The top-down grasp needs S4 at 86° to 112° from straight (section 8); check that this is reachable.
- **S2 behind vertical:** for cubes closer than about 12 cm, the shoulder has to lean backwards (S2 = -14° at r = 10). Check that the real joint allows it, or keep start and target further out.
- **Tick limits:** answered by the controller's guards (section 3, step 6). They are historical, not measured safe travel.
- **S1 axis position:** confirm the 1,5 cm edge distance and that S1 is centred on the 30 cm edge, since every board coordinate depends on it.
- **Which HOME:** the live controller's HOME (S5 = 250, S6 = 550) differs from the documented one (S5 = 480, S6 = 650). Agree on one; every tick offset depends on it.
- **Gripper:** answered, 490 is open (see `controller-live-2026-10-12.md`).
- **Marks on the board:** mark the start and target positions so the cube can be placed at the same spot every time.

## 8. Check in simulation

The simulator in `threejs-sim/` contains the IK from section 3 (`js/kinematics.js`) and a model of the arm built from `arm-dimensions.md`.
Its tests (`node --test threejs-sim/test/*.test.mjs`) set the computed joint angles in the forward kinematics and check where the gripper ends up.
For every reachable test position, the cube-centre point of the gripper lands on the target to within rounding, the gripper points straight down, and the jaws are square to the cube faces.
(The first version of this check used a MuJoCo model in `sim/`; it was removed on 2026-10-12 in favour of the simulator.)

| Cube at (x, y) | Hover | S1 | S2 | S3 | S4 | S5 |
|----------------|-------|----|----|----|----|----|
| (5, 15) | 0 | 18,4° | 17,9° | 74,7° | 87,4° | -18,4° |
| (5, 15) | 5 | 18,4° | 23,3° | 44,4° | 112,3° | -18,4° |
| (-8, 12) | 0 | -33,7° | 10,2° | 84,0° | 85,9° | 33,7° |
| (0, 10) | 0 | 0° | -14,2° | 107,6° | 86,6° | 0° |
| (0, 20) | 0 | 0° | 46,3° | 33,8° | 99,9° | 0° |
| (0, 25) | 0 | unreachable | | | | |

Angles are joint angles of the model, measured from the straight-up zero pose; positive S2 to S4 tilt the arm forward.
They are not yet servo counts, because the angles at HOME are not measured.

The simulator runs the full sequence of section 4 with its own cube physics (gravity, friction, contacts) and checks every plan before it can run.
Its tests move a cube on the board and stack two cubes on another, each ending within 0,9 cm of the target with no contact on the way (`docs/threejs-sim.md`).
Servo dynamics are not modelled: joints follow the controller's commanded trajectory exactly, so this shows that the sequence and geometry work, not that the real servos keep up.
