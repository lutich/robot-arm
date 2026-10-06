# Camera measurement calibration: step-by-step guide

The goal is to measure a rigid target's position and orientation in table
coordinates, check those measurements against physical references, and establish
the target's relationship to the arm's tool point. Keep servo power off and
the arm supported throughout this procedure. Move a separate target by hand;
do not force unpowered arm joints.

The Pi currently provides RGB/depth capture, previews, camera settings and frame
metadata. It does **not** provide marker detection, optical calibration export,
table-frame tracking, tool-offset estimation or an accuracy report. Steps 1–4
are preparation using existing controls; steps 5–10 describe the measurement
prototype and its acceptance procedure. They are not commands for an already
implemented tracker. The agent experiment loop is a later stage.

## 1. Write down the measurement requirements

Before printing or fitting anything, record:

- The part of the table and range of heights the first experiment will use.
- The smallest movement the camera needs to distinguish.
- Acceptable lateral, vertical and orientation errors for that experiment.
- The intended tool point: for example, the centre of the grasp at one fixed
  jaw opening. Record that opening if it affects the fixture.

Agree tolerances from the intended task. Do not adopt a few-millimetre target or
a detection confidence percentage just because an example uses one. If the
requirements are not decided, collect exploratory evidence but do not declare
the system ready for powered calibration.

## 2. Gather and measure the references

Prepare a rigid camera mount, even lighting, a flat rigid table board, a rigid
target carrier, a ruler or calipers, an independently measured grid and blocks
of known height. An angle reference is useful if orientation accuracy matters.

Use a ChArUco board for optical calibration and a board of known marker geometry
for the table reference. The calibration board can later become the table
board, but it must first be free to move through several views. Use a separate
rigid marker or small board as the hand-moved target. Select one dictionary and
record the board dimensions and marker IDs so the detector uses the same
definitions. OpenCV describes [ChArUco calibration](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html)
and [marker generation/detection](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html).

Print at actual size with page scaling disabled. Check several horizontal and
vertical lengths after printing. Measure the marker's black outer square,
including its black border, rather than the surrounding white paper. Mount
the prints flat; record the measured dimensions, dictionary, IDs and carrier
thickness. Printed dimensions establish metric scale, so a printer scale error
affects the resulting pose estimates.

For grid checks, define a physically locatable point on the carrier. Measure
its relationship to the marker origin. A marker placed on a block has a marker
plane height equal to the block height plus the measured mounting offset;
do not silently compare it with the block height alone.

## 3. Fix the camera and check existing image access

With servo power off, mount the camera so the board and target can both be seen
through the intended area and heights. Secure the cable against pulling the
camera. Position the hand-moved target near the centre, edges and raised
locations; check that corners remain sharp and visible without glare.

Open the Pi app's Camera panel. Use View to inspect RGB and depth preview,
then Capture and Download RGB. Download depth PNG when depth evidence is
needed. Preserve both downloads from the same retained capture.

Check these read-only routes in the browser or Swagger documentation:

| Route | What to check |
| --- | --- |
| `/api/state` | Arm unprepared/unarmed, no active movement |
| `/api/camera/status` | Available camera and depth, fresh frame, no pending settings/error |
| `/api/camera/capture` | One RGB/depth pair with run ID, sequence, settings revision, ages and skew |

The active profile is 1280 × 720 at 5 fps, cropped and undistorted RGB. The raw
depth PNG contains uint16 millimetres, with zero meaning unknown; its colour
preview is for display. Camera availability does not establish metric accuracy.
The existing readiness flags intentionally remain false.

If camera-process isolation has just been deployed, first repeat camera-only
capture, concurrent streams, disconnect/reconnect and service shutdown checks
using [deployment](deployment.md). Do not run another camera application against
the same OAK while the Pi app owns it.

## 4. Freeze focus and measurement settings

Use Auto exposure/white balance and Focus once where autofocus is supported
to obtain a clear setup image. Inspect the frame readings. Enter corresponding
manual exposure, ISO and white balance values for review, then Apply. For an
AF-equipped camera, set manual lens position only from an available reading
and verify that the image stays sharp. A missing lens reading needs resolving;
do not guess a focus distance from a lens-position number.

Capture fresh frames after settings stop being pending. Compare available
exposure, ISO, white balance and lens readings with the requested values and
inspect for blur, clipping and flicker. `sent` means dispatch, not verified
sensor application. Manual exposure must itself work with the lighting;
selecting 50 Hz anti-banding only affects Auto exposure.

Record the profile, requested settings and observed values with the evidence.
Keep focus and image geometry fixed during optical calibration and validation.
A focus or geometry change invalidates the measurement configuration until
matching optical calibration has been checked. A reconnect also requires a
fresh settings check because the app restores defaults and changes run ID.

## 5. Calibrate the exact delivered RGB images

The Mac measurement prototype needs a detector/calibration environment whose
versions are explicitly chosen and recorded. This guide does not install
OpenCV or change the robot runtime's dependency locks.

Collect fresh RGB captures of the ChArUco board at varied positions, distances
and tilts, covering the delivered image, with the camera and settings fixed.
Keep the board rigid and still for each capture. Reserve some views for checking
the result; do not use every image to fit. Inspect detected corner IDs and
discard blurred, clipped or incorrectly detected views.

Fit the effective camera matrix and distortion model for these delivered images
using the measured board geometry. Save the calibration, image dimensions,
settings, input images and fit report. Check reprojection errors on reserved
views and look for systematic errors near the edges. The procedure follows
[OpenCV's camera calibration tutorial](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html).

Alternatively, factory calibration can be a starting point only after exporting
the intrinsics for this exact output and confirming its crop/undistortion
geometry. The current HTTP API does not export those intrinsics. Do not simply
resize a sensor matrix or apply the raw sensor distortion coefficients again
to already undistorted images. Use the effective delivered-image model; any
residual distortion fitted from those images belongs to that model.

## 6. Establish the table coordinate frame

Fix the reference board to the table after optical calibration. Mark an origin
and the agreed X/Y directions; define Z upward and lengths in millimetres.
Account for the board/carrier thickness if the coordinate origin is supposed
to lie on the tabletop rather than on the printed board surface. Align this
with the agreed arm/table conventions before later arm fitting.

The prototype detects known board points and estimates their pose in camera
coordinates using the matching optical model. Record the transform and board
definition. With `T_camera_table` mapping table points into camera coordinates,
convert a detected target pose using:

```text
T_table_target = inverse(T_camera_table) × T_camera_target
```

Check the signs by hand-moving the target along marked +X, +Y and upward.
The reported coordinates must increase along the corresponding axes.
[OpenCV's pose documentation](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)
describes the object-to-camera transform returned by PnP.

A table-plane pixel mapping alone cannot determine an elevated target's Z.
Use the calibrated target pose. Keep the fixed board visible to monitor camera
movement; changes or missing board observations must invalidate measurements
under the prototype's checked frame policy, rather than silently redefining
the table coordinates.

## 7. Track the rigid target and establish the tool offset

For each fresh image, detect the target, use its measured geometry and estimate
its pose. Save the image and matching capture metadata with the observation.
Reject ambiguous planar-pose solutions, missing corners and invalid geometry.
Inspect several target orientations for pose flips; a plausible overlay or low
reprojection error alone is insufficient. OpenCV documents both
[marker pose estimation](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html)
and [PnP solutions](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html).

First validate this separate hand-moved target. Then prepare a rigid mounting
relationship to the intended tool point, recording the offset and orientation
with physical measurements. A target on a moving gripper finger needs an
opening-dependent relationship; use a fixed-opening fixture or a rigid body
mount for the first experiment. Attach it only while the unpowered arm is
properly supported, without moving joints by force.

With `T_target_tool` mapping tool coordinates into target coordinates:

```text
T_table_tool = T_table_target × T_target_tool
```

Check the derived tool point against an independently measured fixture point
at several hand-held orientations. If that point shifts unexpectedly, resolve
the offset or pose ambiguity before accepting tool measurements. Calibration
of the camera target alone does not validate the final tool mounting.

## 8. Measure accuracy, jitter and repeatability

Choose the grid positions, heights and orientations before collecting the
acceptance set. Use positions across the intended area, including edges and
the nearest/farthest planned heights. Keep these physical references separate
from the images used for optical fitting or offset adjustment.

At each location, align the carrier's reference point with its independently
measured position, allow it to settle, and collect a stationary series of fresh
frames. Return to selected locations from different directions and repeat.
Compare like points: marker origin against marker-origin references, or derived
tool point against tool-point references. Include carrier thickness and offsets.

Report X/Y/Z bias and error, lateral error, vertical error, stationary spread,
repeat-placement spread, orientation error where a reference exists, and
detection failures. For lateral error use `sqrt(dx² + dy²)`; also report signed
axis errors so a systematic offset is visible. Record measurement uncertainty
in the ruler, grid, block and placement procedure.

If stereo depth will be used for measurements, separately check numeric depth
against measured distances and surfaces over this same region. Alignment and
millimetre units do not establish physical depth accuracy. Depth is optional
for the initial RGB marker-pose prototype.

## 9. Prove that invalid observations are refused

With the arm still unpowered, cover or partially hide the target, blur a capture,
present an unexpected marker, disconnect/reconnect the camera, and exercise
stale/repeated frames in the prototype. Verify that it returns an invalid
observation with a reason and excludes it from calibration samples.

Check camera and board disturbance too. Move one deliberately after preserving
the baseline evidence, verify the prototype invalidates the coordinate setup,
then re-establish and validate it. A single board cannot reveal every movement
relative to the wider table; compare fixed independent table reference marks
and record an operator check before each later batch.

Pin the camera run, settings revision, optical calibration and reference-board
definition for a measurement session. Refuse changes to those identities.
Use acquisition ages and sequences, not a previous screen image or HTTP receipt
time alone. Do not carry observations across reconnection as though they were
from the same measurement session.

## 10. Record the result and decide whether to advance

Keep captures, matching metadata, physical-reference measurements, board/target
definitions, optical calibration, transforms, settings and the accuracy report
together under a named local evidence directory in `artifacts/camera-check/`.
Separate fit evidence from acceptance evidence. Record the tested area and
heights, rejected observations, thresholds derived from the measurements and
remaining limitations. These are camera measurements, not `validated_limits`
for robot joints.

Advance to a few attended arm experiments only when the measured errors and
spread meet the requirements from step 1, the smallest proposed movement can
be distinguished from noise, invalid observations are refused, and the tool
mount/offset has been checked. Camera process isolation also needs its Pi
acceptance check. If any item fails, address that cause and repeat the affected
checks; do not enlarge robot movements to compensate for noisy measurements.

The next stage requires its own bounded executor and checked motion envelope.
Passing this procedure does not implement an agent loop, validate joint travel
limits, or activate an XYZ arm model.
