# Model vs. photo comparison

The three.js model is checked against the real arm one photo at a time.
For each reference photo we estimate the camera pose and the arm's joint angles, render the model from that camera, and lay the render over the photo.
Where they disagree, either the model geometry is wrong or the calibration is wrong; both are listed below.

Tools, all in `threejs-sim/`:

| File | What it does |
|------|--------------|
| `tools/photo-views.json` | One entry per photo: marked points, camera, pose, fitted angles, fit result |
| `js/viewfit.js` | Named model keypoints and the fitter: camera (centre, rotation, focal length) and joint angles by Levenberg-Marquardt on the reprojection error. Pure JS, tested in `test/viewfit.test.mjs` |
| `tools/fit-views.mjs` | Runs the fit for the views in the JSON and writes the result back (`--save`) |
| `tools/compare.mjs` | Renders model, photo and 50 % overlay side by side into `docs/images/compare/`; `--keypoints` also draws the model points (green) and the marked points (red, yellow line = error) |

Nothing here talks to the controller or the camera; `compare.mjs` starts its own simulator server and a headless Chrome.

## 1. Procedure

1. Put the photo in `docs/images/photos/` and add a view to `tools/photo-views.json` with a rough hand-set camera (`pos`, `target`, `fov`) and the pose (`"pose": "PARK"`, or `"counts"`).
2. Render it with keypoints: `node tools/compare.mjs --only <name> --keypoints`. The green labels show where each named point sits on the model.
3. Mark the same points in the photo as `"points": { "<keypoint>": [u, v] }` (photo pixels; a third value is a weight, 0,5 for points you are unsure of).
   A pixel grid over the photo helps to read the coordinates.
   - Mat marker corners `m<id>.<k>` are the most reliable points. Corner order is OpenCV's: for every marker, `k = 0..3` goes (X far, Y left), (X far, Y right), (X near, Y right), (X near, Y left) in the camera table frame.
   - Use the joint axis points `s2_axis`, `s3_axis`, `s4_axis` (the middle between the two visible ends of an axis) and the horn/pin ends. They are schematic values, so they stay right where the drawn brackets are wrong.
   - The tube points (`tube_lo`, `tube_hi`), `s3_back` and the base corners depend on drawn geometry that is known to be off (section 3). Leave them out, or give them a low weight.
   - Arm tags: `t<id>` (centre) and `t<id>.<k>` (black-square corners in the order OpenCV and `detectArmTags` return them), from `ARM_TAGS` in `config.js`. Use the detected corners directly.
   - At least 4 points, spread over the image; better 8 or more, including some that are not in one plane.
4. Fit: `node tools/fit-views.mjs --only <name>`. Options go in the view's `"fit"`:
   `fov` (prior on the vertical field of view; phone photos have no EXIF: 61,7° in portrait, 48,3° in landscape for the main camera, from the marker fit of `park-top-view.jpg` (photo removed 2026-10-13)), `fitD` (also fit the mat offset `d`), `free` (joints to fit, default S1 to S4; `[]` = camera only), `start` (angles to start from), `skip` (a reason to leave the view out).
5. Look at the worst residuals it prints. Over 30 px usually means a wrongly identified point, or a point on geometry the model draws wrong.
6. Photos of the same pose share one set of angles: `node tools/fit-views.mjs --joint` fits all photos with the same `pose` together (one camera per photo, one set of angles and one mat offset `d`). This is what is saved for the PARK photos.
7. `--save` writes camera, angles and the fit result into the JSON. Then `node tools/compare.mjs` renders all views; `compare.mjs` draws `angles` exactly, so the overlay shows the fitted pose and not the rounded counts.

## 2. Results, 2026-10-13

Ten views, all in the PARK pose (counts 330 / 540 / 580 / 195 / 480 / 650).
Before = the hand-set cameras and the model PARK angles (`docs/images/compare/before/`), after = fitted (`docs/images/compare/`).

| Photo | Camera | RMS | Match |
|-------|--------|-----|-------|
| `park-top-view.jpg` (photo removed 2026-10-13) | joint fit, 16 marker corners + 2 axes | 5,9 px | **Match.** Markers, S4 bracket and elbow box line up |
| `park-top-view-forearm.jpg` (photo removed 2026-10-13) | joint fit, 10 marker corners + 3 axis points | 9,2 px | **Match** for markers and joints; the S4 bracket differs (section 3) |
| `park-front.jpg` | joint fit, 7 marker corners, base, S4 axis, S5 horn, pads | 14,9 px | **Match** for base, markers, S5 horn; gripper and wrist drawn wrong |
| `park-gripper-on-base.jpg` (photo removed 2026-10-13) | joint fit, 6 marker corners, base front corners, pads | 18 px | **Partial.** The real base is taller than drawn; base back corners left out (100 px off) |
| `park-side-pca9685.jpg` (photo removed 2026-10-13) | joint fit, marker 0, base, S3/S4 pins, S5 horn, pads | 19,9 px | **Partial.** Joints line up; the S5 horn is 46 px off (wrist geometry) |
| `pca9685-on-upper-arm.jpg` | camera only, 4 PCA9685 board corners, pose from the joint fit | 18 px | **Match** for the board after the PCA9685 fix (section 3) |
| `pca9685-closeup.jpg` (photo removed 2026-10-13) (new view) | camera only, 3 board corners | exact (3 points) | **Match** for the board; only 3 points, so the camera is a guess with the fov prior |
| `gripper-front-closeup.jpg` | hand-set, fit skipped | – | Not fitted: only base points, and the drawn base is too low, so the fit put the camera under the board |
| `base-s1-s2-from-back-left.jpg`, `pca9685-cables-elbow.jpg` | hand-set | – | Not fitted: blurred close-ups without markers or reliable corners. Pose from the joint fit |
| `arm-tags-106-107-109-front-s6-right-close.jpg` | S1 to S5 free, marker 1, tags 109, 106, 107, S5 hub | 12,4 px | **Match** for the S6 servo (right of the gear plate seen from the front) and the tags 106/107. The wrist and its tag 109 still differ (S5 servo bracket) |
| `arm-tags-106-107-109-front-s6-right.jpg` | S1 to S4 free, S5 from the close-up, markers 0 and 1, tags 109 and 106, S5 hub | 10,2 px | **Match** for the S6 servo and tag 106. The gripper hangs straight in the photo but tilted in the fit: the wrist tag 109 is placed only roughly |

### Arm pose at PARK

Joint fit of the five photos with markers or base points:

| | S1 | S2 | S3 | S4 | Mat offset d |
|--|----|----|----|----|--------------|
| Fitted from the photos | −1,8° | −33,3° | 109,9° | 107,0° | −1,76 cm |
| Model PARK from counts (`config.js`) | 0,2° (was −3,3°) | −27,7° (was −40,2°) | 107,4° (was 128,4°) | 107,0° (was 93,0°) | −2,5 cm (estimate) |

- Check: with the fitted angles the gripper pads end at z = 3,1 cm (pad centre), so the 1,2 cm pads rest on the base top plate at 2,5 cm, as in every PARK photo. With the model angles they are at −0,2, inside the board.
- Taken alone, this one pose gives these HOME angles (`qHome`) if the directions and counts per degree in `config.js` are right: S1 1,5 (now 0), S2 12,9 (now 6), S3 43,5 (now 62), S4 98 (now 84). **Not applied:** one pose cannot separate the offset from the scale, and the current values come from the team's taught demo. A second photographed pose (HOME) would.
- S5 is not observable in these photos (the jaws are closed); it stays at the model value.
- `d` = −1,76 from the joint fit, against the −2,5 estimate from the overhead photo. Still not measured (`HANDOVER.md` open item 3).

## 3. Model differences found, and what was fixed

| Part | Photo | Model | State |
|------|-------|-------|-------|
| PCA9685 and pink foam | On the upper arm's −X side plate, board facing out (−X), long side along the link; servo headers on the back edge with channel 0 at the bottom, power terminal on the front edge, I2C header on the bottom edge, two zip ties | Was on the back of the upper arm | **Fixed** (`js/model/arm.js` `buildPca`, cables re-routed in `cables.js`, collision box in `collision.js`, keypoints `pca*`) |
| PARK angles | Pads on the base top plate | Pads 0,2 cm in the board | Fitted angles saved per view; calibration not changed (section 2) |
| Base | Taller: the top plate is visibly higher than drawn (`park-gripper-on-base.jpg` (photo removed 2026-10-13)) | H = 2,5 | Open. Try H ≈ 3,3 with S1 shaft and H_S2 = 9,6 unchanged (S1 servo sinks into the base) |
| S4 | Servo body lies along the forearm in a bracket box; the tube is shorter | Servo across the forearm, long tube | Open (`park-top-view-forearm.jpg` (photo removed 2026-10-13)) |
| Gripper fingers | Straight bars | Curved claws | Open (`park-front.jpg`, `gripper-front-closeup.jpg`) |
| S6 servo and tags 106/107 | Seen from the front with the gripper hanging down and the jaws closing toward the camera: S6 body on the right of the gear plate, reaching back, its cable end facing the camera (tag 106 on a clear spacer), tag 107 on the face toward the wrist. Top face 1,5 cm below the S5 hub, tag 106 centre 2,05 cm off the S5 axis and 2,95 cm below the hub (tag poses from their corners, OpenCV IPPE, against the hub; `arm-tags-106-107-109-front-s6-right*.jpg`) | Servo and tags were 0,9 cm too low; 106 0,45 cm too far out | **Fixed** 2026-10-13 (`config.js` `GRIPPER.gear`/`outer`/`tip` and `ARM_TAGS`; `model/gripper.js`, `collision.js`, `cables.js` follow). In these photos S5 is about −88° (model angle), i.e. HOME (S5 = 250). S5 calibration changed to match: qHome −90, dir +1 (was +90, −1), so HOME shows S6 on the right while PARK (480, model 0, `park-front.jpg`) is unchanged |
| Wrist / S5 | S5 horn 46 px off in the side view | | Open; check after the S4 fix |

After each open fix: refit (`--joint`) and render again; the RMS values above are the baseline.
