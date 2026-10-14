# Two cameras: tracking the arm and locating cubes in 3D

Since 2026-10-13 two cameras watch the mat (`images/photos/setup-two-cameras-*.jpg`):

| Camera | Where | API (controller, GET only) |
|--------|-------|----------------------------|
| OAK | on an articulated stand at the desk edge | `/api/camera/status`, `snapshot.jpg`, `stream.mjpg`, `capture` |
| Webcam | clipped to the top of the monitor beside the desk | `/api/camera2/status`, `snapshot.jpg`, `stream.mjpg`, `capture` |

The second camera's routes were read from the controller's `/openapi.json` on 2026-10-13. They are not yet in `lutich/robot-arm` on GitHub. `server.mjs --camera http://10.102.4.103:8765` forwards both, read only. Without it, the OAK falls back to the saved frame; for the webcam, load an image file.

AprilTags (family 36h11, 19 mm black square, white margin) on the arm, in two rigid parts (`images/photos/arm-tags-106-107-109-*.jpg`, `arm-tags-108-109-*.jpg`). Tags on one part move together and act as one object. One camera may see one tag of a part and the other camera another, and the views still meet in one pose. Each part has one degree of freedom relative to the part before it.

| Part | Moved by | Tag | Where |
|------|----------|-----|-------|
| Wrist (S5 servo + its bracket) | S1–S4, turns about S4 | 109 | S5 servo's wide face |
| | | 108 | S5 bracket's side plate, horn side (+X) |
| Gripper (S6 servo + jaws) | S1–S5, turns about S5 | 107 | S6 servo, face toward the wrist |
| | | 106 | S6 servo, cable end |

The numbers (centre, normal, up per tag, in its link frame) are in `js/config.js` `ARM_TAGS`.

How the layout was measured (in three steps, each better than the last):
- **From the arm model and the photos:** 109 and 107 were placed from the model, and 108 and 106 relative to them from the close-up photos.
- **Joint axis, 2026-10-13:** the S5 joint can only turn the gripper about one axis fixed in the wrist. Three poses with S5 120° apart (two live snapshot pairs and the close-up photos; the team had moved the arm in between) were fitted with `js/jointfit.js estimateJointAxis`.
  - The axis came out 10,6° and 0,5 cm from the model's.
  - The corrected layout brought the wrist + gripper pair fit from 17–50 px down to 3–9 px.
- **Still open:** how far along the S5 axis the gripper tags sit relative to the flange cannot be seen from joint motion. The fit puts 107 about 2,6 cm below the flange. **2026-10-13:** two front photos (`arm-tags-106-107-109-front-s6-right*.jpg`) give it directly from the tag poses against the S5 hub: 107 1,5 cm below the hub, 106 2,95 cm below and 2,05 cm off the axis. The model (S6 servo and both tags) follows them now. **What to measure and where: [`measure-arm-tags.md`](measure-arm-tags.md).** A fourth live pose (the arm turned to its left) confirmed the layout to 0,5° / 0,14 cm on the S5 axis; across leave-one-out fits, tag 107 moves by up to about 1 cm, which the measurement settles.

Older notes on the layout:
- 109 and 107 are placed from the arm model.
- 108 and 106 are placed relative to them from the close-up photos. In each photo both tags' poses come from their corners (OpenCV IPPE), and their relative pose barely depends on the phone's focal length. The three gripper photos agree to 2 mm.
- Each tag's in-plane turn on its face comes from the same photos, and the tracker checks it in every frame (it reports a tag "stuck on turned" when it differs).

Earlier prints (72/71, then 103/100 and two without a white margin) were replaced. **Tags need a white margin of at least 1/8 of the black square and must sit flat on one face.** The old photos were removed.

**Overlay.** "Tags & parts" in the 3D view, and the same checkbox in the Camera tab, colours each tag by its part, joins the tags of one part and links them to the joint axis that moves the part. The Arm tracking box lists every tag with its part, the joints that move it and the cameras that saw it.

**How big must the tags be** (live test, 2026-10-13: a strip of five tags 105–109, 18,9 mm black square with a white margin, held about 18 cm above the mat):
- Both cameras found all five, OpenCV and the browser detector alike. They appeared 23 to 39 px wide in the 1280 × 720 frames.
- Triangulated from both cameras, the spacing came out as 35,4 / 35,0 / 35,0 / 34,6 mm and the side as 18,9 ± 0,3 mm.
- The same frames found tag 103 on the arm only in the webcam, and tag 100 in neither (smaller in the image, at a steep angle).
- So: about 20 px across in the image is the lower limit. 19 mm works for tags facing the cameras; 25 to 30 mm (with a 4 mm margin) is safer on the servo faces, which leave room for that.

## How it works

All of it runs in the browser (Camera tab), in plain JS. The mat markers are ArUco 4×4; the arm tags are AprilTag 36h11 (`js/apriltag36h11.js`, all 587 codes). Both are read by the same detector, `js/aruco.js`.

1. **Calibrate each camera** from the mat markers in its own frame, as before ("Calibrate all cameras").
   - Each camera keeps its own fit, stored per browser.
   - The 3D view shows both cameras and their fields of view, and the "Webcam" view button looks through the second one.
2. **Arm tracking** ("Track the arm", `js/armtrack.js`):
   - The tags are detected in every camera's frame.
   - A tag seen by both cameras is **triangulated**: the viewing rays from both cameras are intersected (linear least squares). This is its 3D position, independent of the arm model.
   - The joint angles are then fitted so that the model's tags project onto all the tag views (Levenberg-Marquardt). S5 is fitted only when tag 71 (on the hand) is seen.
   - The tracked pose is shown as a ghost arm. The table "tracked − commanded" is what the HOME calibration still gets wrong (`HANDOVER.md`, open item 1).
3. **Cubes in 3D** ("Find cubes", `js/locate3d.js`):
   - Cubes are found by colour in every camera's frame.
   - For each colour the blobs are paired across the cameras, and their viewing rays are intersected.
   - Each ray passes through the centre of the faces that camera sees, not through the cube's centre, so every ray is moved back by that offset before intersecting. A cube under another shows no top face.
   - The result is the cube's centre with its real height: the stack level, or "between levels" for a cube in the gripper or tilted. The gap between the rays shows how well they met.
   - **Cubes are no longer assumed to lie on the mat.** With only one camera, the old mat-plane estimate is used.
   - "Put them into the sim" places the cubes at their stack levels in the shared simulation.

### Rigid parts, both cameras combined (`js/rigidtrack.js`)

- **One tag in one camera:** its pose (rotation + position) comes from its four corners. The homography decomposition is refined on the reprojection error, from the solution and from its mirror image (a small tag seen at an angle has two nearly identical poses), and the better one is kept.
- **The part's pose** is fitted to all its tags' corners in all cameras together.
  - Each camera is weighted by its calibration quality (`camWeight`: 1 / px rms from the mat markers), so the OAK (about 0,4 px) counts more than the webcam (about 2 px).
  - A tag seen by both cameras gives the parallax that fixes its depth; one camera alone fixes the depth only through the tag's size.
- **Small tags** (under about 25 px in the frame) are also searched in a 2× enlarged crop around where the arm is expected. That finds 107 in the OAK and 108 in the webcam, which a full-frame search misses.
- **Display:**
  - "Estimated parts" draws each part where its tags alone put it, as a wireframe over both camera images.
  - "Learn tag layout" measures where each seen tag sits on its part.
  - "Cameras" in the main toolbar shows both cameras large instead of the 3D view; ⛶ makes one camera, or both, full screen.

### Joints: tags across parts (`js/jointfit.js`)

Each joint lets the child part only turn about one axis fixed in the parent part, so tags on neighbouring parts are combined with the joint's single degree of freedom:
- **Per frame** ("Wrist + gripper together" in the Arm tracking box):
  - The wrist pose (6 numbers) and S5 (1 number) are fitted to the tags of both parts in all cameras.
  - A gripper tag helps fix the wrist and vice versa: 7 unknowns instead of 12.
  - If this fit is much worse than the two parts alone, the tag anchors or the axis are off.
- **Across frames** (Record pose, Fit S5 axis):
  - Record 3 or more poses with different S5 angles. The child's motion relative to the parent then gives the real axis.
  - The result: the tag layout of both parts corrected to it (used at once, and copied for `config.js`), and S5 measured against commanded (slope = direction, offset = the error of S5's angle at HOME).
  - The same works for S4 and further joints once the forearm and upper arm carry tags.
- **The joint-chain fit** (all joints from the base) then uses the corners in the turn each tag is found in.

## Checks

| Check | Result |
|-------|--------|
| Unit tests (`test/armtrack.test.mjs`, `test/locate3d.test.mjs`) | Triangulation exact without noise, 3 mm with 0,5 px noise. Joint angles recovered to 0,2° from the two tags. Cubes on the mat, stacked (levels 1, 2) and in the air located to under 4 mm |
| Detector on the photos | Tag 72 found in all three close-ups, and the printed sheets' tags in the overviews. Tag 71 needs a small threshold window (it sits on the black servo with a thin white margin); the tracker tries four settings |
| In the page, frames rendered by the simulator from two cameras (headless Chrome) | Both cameras recovered from the mat markers to 4 to 7 mm. Tag 72 triangulated within 1 mm of the model. Joint angles within 1,3° of the commanded pose. Stacked blue-on-orange found at levels 0 and 1, a single cube to 1 mm in height |
| Live snapshots of both cameras, 2026-10-13 (one GET each), tags 106–109 | Single-tag poses: 0,1 to 1,4 px. Wrist part: 2,5 px (109 in both cameras). Gripper part: 1,5 px (107 in both, 106 in the webcam). The fitted gripper's tool axis lines up with the wrist's to 0,998 and is turned 22° about S5, as the joint allows, but sits about 4 cm from the model's place: the anchor positions of 109 / 107 on their parts are still model guesses. The joint-chain fit to the commanded pose is off by 30 to 60° on S2 to S4: that is the uncalibrated HOME angles |
| Live check 2026-10-13 (one snapshot per camera, one state read; arm at 202 499 505 287 381 650) | OAK: tag 107, webcam: tag 109, mat markers 0.6 / 1.5 px. Model at the commanded counts, old HOME angles: 50 / 52 px off, gripper tips 4.4 cm from the real ones (triangulated). S1 to S5 fitted to both tags' corners: 2.8 px, tips 2.4 cm. With the mean of this pose and the PARK photos written into `qHome` (S1 to S4, see `config.js`): 13 / 16 px and 2.9 cm at the commanded counts, the taught demo still holds. Views: `live-2026-10-13-*.jpg` in `tools/photo-views.json` |
| After the axis calibration, both live snapshot pairs in the page | Wrist 1,3 / 2,5 px, gripper 2,3 / 1,8 px, wrist + gripper together 4,0 / 3,0 px (was 17,7). The joint-chain fit puts each part within 0,2 to 0,9 cm of its tag-only estimate |
| Known limit | A cube half hidden in one view (behind the gripper) is placed wrong; its rays then miss by several mm and it is reported as "between levels", not placed |

Real frames from both cameras have not been run yet. They need the server started with `--camera http://10.102.4.103:8765` (one snapshot per camera on request, or live video).

## Next

- **Anchor and HOME calibration from a few poses:** move S4 and S5 through a few angles (with someone at the arm). In each pose fit the two parts (above). The wrist part must turn about one fixed axis, and the gripper about one axis relative to it. These axes fix the anchor positions of all tags, and then the angle at HOME and the direction of S4 and S5 (`HANDOVER.md`, open item 1). S1–S3 follow the same way with a tag on the upper arm or forearm.

- Measure the tag positions and sizes on the arm. Add a third tag on the upper arm or forearm for a better-determined pose.
- Collect tracked − commanded over a few poses and fit the HOME angles and directions from them (Calibration panel).
- Add the second camera to the simulator's own robot API (`/api/camera2/*` on the simulated backend: rendered frames), so other teams can test against it.
