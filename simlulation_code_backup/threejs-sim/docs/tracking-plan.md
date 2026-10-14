# Plan: tracking the blocks and the arm with the camera

Task: move and stack the coloured cubes, picking the cube that is asked for ("put blue on orange").
A cube can stand on the mat or on another cube, so its height is not always zero.

Constraints:

- One RGB camera, the OAK on the controller (`controller-live-2026-10-12.md`).
- **No depth.** The OAK stereo depth is not usable on this setup, so everything comes from the RGB image and known geometry.
- A slow update rate is enough: one frame every 5 seconds.

## 1. What we know without depth

| Known | Used for |
|-------|----------|
| The printed mat with six ArUco markers at known positions (280 x 360 mm) | Camera model per frame |
| Cube size, 26,5 mm | Stack heights, cube centre, plausibility checks |
| Cubes are axis-aligned boxes resting on the mat or on each other | Heights are multiples of 26,5 mm |
| The arm's link lengths and the commanded servo counts | Where the arm should be (`threejs-sim/`) |

## 2. Camera model from the mat, every frame

The saved homography in the controller repo (2026-10-11) no longer fits: on 2026-10-12 the markers were off by 5 to 39 mm.
The camera or the mat has moved since then.
So `vision/blocks.py` fits the camera from the visible markers in every frame, instead of trusting a stored calibration.

It fits a full pinhole camera (focal length, principal point, pose) to the marker corners, not just a mat homography.
With the full camera, an image point can be mapped onto any horizontal plane, for example the top of a stack two cubes high.

Results on the real frames of 2026-10-12:

| Value | Result |
|-------|--------|
| Reprojection error | 0,86 px RMS |
| Focal length | 952 px, a 68° horizontal field of view, typical for the OAK RGB camera |
| Camera position | 481 mm along X (12 cm beyond the far edge), Y = -8 mm, 372 mm above the mat |
| Same fit on two frames 2 minutes apart | Camera centre within 1 mm |

At least two markers must be visible; four or more spread over the mat are better.
The arm or cubes may cover some markers, which is fine as long as enough remain.

## 3. Finding the cubes

`vision/blocks.py`, per frame:

1. **Colour:** segment each cube colour in HSV. Only orange is tuned so far; red, yellow, green and blue have untested starting ranges.
2. **Stack level:** a cube whose bottom edge in the image borders another cube's colour rests on that cube, otherwise on the mat. Levels follow by counting down the chain.
3. **Position:** map the cube outline onto the plane of its bottom face (Z = level x 26,5 mm). Outline points that are really higher land further from the camera, so the points with the largest X form the front-bottom edge. Centre = middle of that edge, moved half a cube away from the camera. The edge also gives the cube's yaw.
4. **Checks from the known size:**
   - The front edge must measure 26,5 mm ± 4 mm. Otherwise the cube is partly hidden, for example by the gripper, and the result is flagged `valid: false`.
   - For cubes with nothing on top, a second level estimate projects a full 26,5 mm cube for each possible level and compares it with the blob. A cube one level off comes out about 7 % too large or too small. If this disagrees with step 2, the cube is flagged.

Output per cube: colour, level, the cube it rests on, X and Y in mm, bottom and top height, yaw, edge length and the validity flag.
`--s1-offset-mm` also prints the position in the board frame of `arm-dimensions.md`.

### Results

| Test | Result |
|------|--------|
| Real frame, orange cube on the mat | Level 0 by both methods, front edge 26,1 mm (real 26,5), wireframe matches within about 3 to 4 mm |
| Real frame, orange cube held in the gripper | Edge 11 mm, correctly flagged as hidden |
| Synthetic render (`vision/test_synthetic.py`): orange, blue on orange, yellow on blue, green alone | All levels right, position error 0,6 to 2,1 mm |

The synthetic test rendered the mat and cubes in MuJoCo through the camera fitted to the real frame, then ran the detector on the render (`vision/test_synthetic.py`, removed on 2026-10-12 together with the MuJoCo code; the result above stands).
It checks the geometry, not the colour thresholds or real lighting.

### Limits

- A cube in the gripper is in the air at an unknown height. Use the arm's commanded pose for it, not the camera.
- The top face of a covered cube is hidden, so the size check only works for the top cube of each stack.
- Two cubes of the same colour touching each other merge into one blob.
- A cube hidden behind a taller stack, seen from the camera, is not found.

## 4. Tracking the arm

Not built yet. Options, in the order we suggest:

1. **ArUco marker on the gripper.** A small marker (about 20 mm, an ID above 5 so it does not clash with the mat) on the back of the gripper, facing the camera. With the fitted camera, `solvePnP` on its corners gives the gripper's full 3D pose. That works in the air too, which the cube method cannot do.
2. **Calibrate with it.** Command a set of poses, record the marker pose for each, and fit:
   - the joint angle at HOME and the counts per degree for each servo, the main open item in `pick-and-place-plan.md`;
   - the S1 axis position on the mat (the offset `d`), from the circle the gripper draws when only S1 moves.
3. **Render and compare.** For each commanded pose, render the three.js model (the simulator's Camera tab already renders it through the fitted camera) through the fitted camera and compare it with the dark arm pixels in the image. No marker needed, and at one frame every 5 seconds there is plenty of time. More work and less precise than a marker.

Monocular depth networks are not needed: the cube heights come from the known size, and the arm's from its kinematics.

## 5. How it fits into stacking

Per command, such as "put blue on orange":

1. Take a frame, find all cubes and their levels.
2. Pick: the blue cube, which must be a top cube. Grasp height = its centre, `(level + 0.5) x 26,5 mm`.
3. Place: on top of orange's stack. Release height = `top_z + 13,25 mm`, at the X and Y of the top cube of that stack.
4. Convert both positions to the board frame (needs `d`) and run the IK (`threejs-sim/js/kinematics.js`, `ikTopDown`) with the cube centre at that height.
5. After placing, take a new frame and check that blue is now one level above orange. If not, retry or stop.

### Reach for stacking

Higher cubes leave less reach for a top-down grasp (`threejs-sim/js/kinematics.js`, `maxReach`):

| Cube level | Cube centre height | Max distance from S1, grasp | Hover 2 cm | Hover 5 cm |
|------------|--------------------|-----------------------------|------------|------------|
| 0, on the mat | 1,3 cm | 21,1 cm | 20,1 cm | 18,0 cm |
| 1 | 4,0 cm | 19,7 cm | 18,3 cm | 15,6 cm |
| 2 | 6,6 cm | 17,8 cm | 15,9 cm | 12,0 cm |
| 3 | 9,3 cm | 15,2 cm | 12,6 cm | 5,7 cm |

Build stacks close to the arm, and use a short hover (about 2 cm) above level 1 and higher.

## 6. Running it

```powershell
.\.venv\Scripts\python.exe -m pip install opencv-python          # once, next to mujoco and numpy
.\.venv\Scripts\python.exe vision\blocks.py docs\images\photos\controller-camera-2026-10-12.jpg --out annotated.png
.\.venv\Scripts\python.exe vision\blocks.py --url http://10.102.4.103:8765 --every 5
.\.venv\Scripts\python.exe vision\test_synthetic.py
```

`--url` only uses `GET /api/camera/snapshot.jpg`; it never sends anything to the arm.
The mat layout is read from the controller repo's `config/camera-table-xy/calibration.json`, cloned next to this repo; `--calibration` points elsewhere.

## 7. Next steps

- [ ] Take one frame with all real cube colours on the mat and tune the HSV ranges.
- [ ] Take one frame with a real stack to check the level detection outside the simulation.
- [ ] Stick an ArUco marker on the gripper and record poses for the arm calibration (section 4).
- [ ] Measure `d`, or fit it from an S1 sweep with the gripper marker.
