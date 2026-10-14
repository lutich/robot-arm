# Recalibrating the arm model from camera captures

The simulator tracks the real arm from AprilTags stuck on it, seen by two cameras that calibrate themselves from the ArUco markers on the mat. For that to put the gripper in the right place, the model has to know:

1. **Where each mat marker is** (so the cameras can calibrate).
2. **Where each arm tag sits and which joints move it** (so the tags can be turned into joint angles).
3. **Where the grip point is** relative to the tags (so the joint angles turn into a cube position).

Item 1 and the rough layout of item 2 come from measurements and photos. The fine values of items 2 and 3 come from **captures**: frames of both cameras taken while the gripper holds a cube on a known spot. This guide covers all of it, so the calibration can be redone after tags, markers or cameras change.

Files: `threejs-sim/tools/capture.mjs` (captures), `threejs-sim/js/recalib.js` (the fit), the Camera tab section "Recalibrate from captures" (runs it), and `threejs-sim/js/config.js` (where the results go).

## When to recalibrate

| Change | What to redo |
|--------|--------------|
| A camera was moved | Nothing: each camera recalibrates from the mat markers in every frame |
| New or reprinted mat, other mat markers | Step 2, then captures (step 3) and the fit (step 4) |
| A tag was moved, replaced or added on the arm | Step 1 (photos and a rough layout), then steps 3 and 4 |
| The gripper or wrist was rebuilt | Step 1, then steps 3 and 4 |
| Tracking puts the gripper visibly off the real cube | Steps 3 and 4 |

## Step 1: tags on the arm (photos first)

The fit can only refine a layout that is roughly right, and it cannot work out **which joints move a tag**. That comes from photos.

1. **Photograph each tag** with the joints around it. Take at least a front, a side and a top view, with the gripper hanging straight down. Save them as `docs/images/photos/arm-tags-<ids>-<view>.jpg`; there are examples in that folder.
2. **Decide the rigid part.** A tag belongs to the part it is screwed or stuck to. The parts are in `ARM_PARTS` in `config.js`:
   - **`wrist`**: the S5 servo and its bracket. S1 to S4 move it, and it turns about S4.
   - **`hand`**: the S6 servo, the gear plate and the fingers. S1 to S5 move it, and it turns about S5.

   A tag on the forearm or upper arm would need a new part (`elbow` or `shoulder`) in `ARM_PARTS`, in `js/tagoverlay.js` and in the fits.
3. **Enter the tag** in `ARM_TAGS` in `config.js`:

   ```js
   { key: 'hand-end', id: 106, link: 'hand', centre: [0.6, -2.05, 2.95], normal: [1, 0, 0], up: [0, 0, 1], size: 1.9, on: 'S6 servo, cable end' },
   ```

   | Field | Meaning |
   |-------|---------|
   | `id` | AprilTag 36h11 id. It must be unique on the arm and must not clash with the mat (ArUco 4x4, ids 0 to 5) |
   | `link` | The part (frame) it moves with: `wrist` or `hand` |
   | `centre` | Centre of the **black square**, in cm, in that part's frame (below) |
   | `normal` | The direction the printed side faces, in the same frame |
   | `up` | The direction of the tag's printed top edge. A guess is fine: the tracker detects a tag stuck on turned by 90° |
   | `size` | Side of the black square, in cm (measure it; the white margin does not count) |

   **Frames** (`js/kinematics.js` `fk`):
   - **`wrist`**: origin on the S4 axis; Z runs from S4 toward the S5 hub (the hub is at Z = `ARM.L4` = 6,7 cm); X along the S4 axis.
   - **`hand`**: origin at the S5 hub (flange); Z along the tool, away from the wrist; X is the axis along which the jaws close.

   Seen from the front with the gripper hanging down and the jaws closing toward you, hand +X points at you, and hand −Y is to your right, where the S6 servo is.
4. **Check the layout.** Use `node tools/compare.mjs` with the tag points `t<id>` and `t<id>.<k>` (`docs/model-photo-comparison.md`), or "Tags & parts" in the 3D view. A few millimetres off is fine: step 4 refines it.

## Step 2: mat markers and cameras

- The mat carries ArUco markers (dictionary 4x4_50). **`MARKER_CORNERS` in `js/calib.js`** lists each marker's four corners in the camera table frame:
  - units are mm, with the origin at the mat's near edge on the arm's centre line;
  - +X points away from the arm and +Y to the arm's left;
  - the corners are in OpenCV order.
- For a new mat or other markers, update `MARKER_CORNERS`. If the dictionary changes, also update `js/aruco.js`. Then check "Auto-calibrate" in the Camera tab: it should report under 2 px rms for each camera.
- The cameras calibrate themselves from every frame, as long as each one sees at least 2 markers (8 corners); 3 or more spread over the mat are better. The camera positions in `CAMERAS` in `config.js` are only first guesses.
- `MAT.sToEdge` (`d`) is the distance from the S1 axis to the mat's near edge. Step 4 fits it.

## Step 3: captures with a known grip point

Each capture is one frame from each camera, taken at the same moment, while the gripper **holds a cube whose position is known**.

**Where to put the cube** (centre of the cube, mm):

| Spot | `--at` | Position |
|------|--------|----------|
| On a mat marker (put the cube centred on it) | `marker2`, `marker3` | 160 / +110, 160 / −110 (any id in `MARKER_CORNERS`) |
| Mat centre | `300,0` | 300 / 0 |
| A grid crossing | `x,y` | read it off the printed grid |
| On another cube | add `--level 1` (2 for a third cube) | the centre is 13,25 + 26,5 × level mm high |

**How to take them:**

- Turn the arm by hand (the servos can be powered off) or move it with the controller, and close the gripper on the cube.
- **Both cameras** must see at least one arm tag each, and at least 2 mat markers.
- Keep hands and cables off the tags and markers. A hand on the arm is fine if it covers nothing.
- **Vary the poses**: left, middle and right (S1); near and far (reach); on the mat and stacked; the wrist turned (S5). Take at least 6 captures, better 9 or more.
- Run one command per capture (read only: one GET per camera and one for the arm state):

  ```bash
  node threejs-sim/tools/capture.mjs --at marker3 cube on marker 3
  node threejs-sim/tools/capture.mjs --at marker2 --level 1 stack on marker 2
  node threejs-sim/tools/capture.mjs --at 300,0 mat centre
  ```

- Each capture is a folder `docs/images/calibration/<date>-<time>-<name>/` with `oak.jpg`, `webcam.jpg`, `state.json` and `info.json`. The `target` in `info.json` holds the cube centre. You can add or correct it by hand later; `vision/blocks.py` on the OAK frame measures where a cube really stands.
- The joint counts in `info.json` are **not used** here: with the servos off they do not match the pose.

## Step 4: the fit

1. Start the simulator with `npm run lab` (in `threejs-sim/`), open it, go to the **Camera** tab and open **Recalibrate from captures**.
2. Choose the values to fit. All of them is the default:

   | Value | What it moves |
   |-------|---------------|
   | wrist tags X / Y / Z | All tags on the wrist part together (wrist frame) |
   | gripper tags X / Y | All tags on the gripper part together, sideways (hand frame) |
   | grip length | S5 hub to the centre of a held cube (`ARM.G_BLOCK`) |
   | mat offset `d` | S1 axis to the mat edge (`MAT.sToEdge`) |

   The gripper tags' **height along the tool cannot be fitted**: moving them along the tool looks exactly like a different grip length. It stays as measured in step 1.
3. Press **Recalibrate from captures**. For every capture with a target, the page:
   1. calibrates each camera from its mat markers;
   2. finds the arm tags, searching around and above the target spot, also enlarged, and in the whole frame;
   3. fits the chosen values together with each capture's five joint angles. The fit is Levenberg-Marquardt on the tag corners in px plus the grip point in mm, weighted 1 px per mm.
4. Read the table. "Grip error before / after" is measured the way live tracking would see it: each capture's angles are fitted **to the tags only**, then the model's grip point is compared with the target. Accept the values when:
   - the mean grip error drops, and most captures improve;
   - the tag fit stays at about 3 px or less;
   - the changes are plausible: millimetres for the tags, and a grip length that matches a ruler (hub to the middle of the fingers' contact pads).

## Step 5: keep the values

- **Use now** applies the values to tracking and planning in this page until it is reloaded. The drawn tags move only after a reload.
- **Copy for config.js** gives the new tag centres, `ARM.G_BLOCK` and `MAT.sToEdge`. Enter them in `threejs-sim/js/config.js`, then:
  - run `node --test threejs-sim/test/*.test.mjs`;
  - reload the page and check live tracking ("● Live tracking": the tags on the model should sit on the tags in the camera views);
  - commit the captures together with the new values.
- The grip length and `d` also change the IK, so planned moves change. Try them in the simulation before running anything on the real arm.

## What it does not do (yet)

- **HOME angles and servo directions** (`qHome`, `dir`) need captures where the counts match the pose, i.e. with the arm moved by the controller. With those, the same captures could fit `qHome` per servo: the tracked angle minus the commanded one, as in `docs/two-camera-tracking.md`.
- **Tag rotations** are not fitted, only shifts. A tag stuck on at an angle needs its `normal` and `up` from the photos.
- **Accuracy** depends on how exactly the cube stands on the spot (about 2 to 3 mm by hand) and on the cameras: tags under about 25 px are found less reliably.

## Results, 2026-10-13

9 captures by hand: marker 2, marker 3, the mat centre and the centre line near X 200, half of them stacks (`docs/images/calibration/`). Tags found by the page's detectors:

| | Grip error, mean | Range |
|--|-----------------|-------|
| Model as it is | 11,2 mm | 5 to 20 mm |
| After the fit (all values) | 6,3 mm | 2 to 12 mm |

The fitted changes:

| Value | Change |
|-------|--------|
| Grip length | 11,00 → 10,05 cm |
| `d` | −2,5 → −1,2 cm |
| Wrist tags | −2,2 / +0,2 / −0,5 mm |
| Gripper tags | −1,4 / −1,2 mm |

Not yet copied into `config.js`: the grip length comes from the schematic. Check it with a ruler first.
