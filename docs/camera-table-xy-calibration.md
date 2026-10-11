# Camera table XY calibration

The saved [calibration](../config/camera-table-xy/calibration.json) maps delivered
RGB pixels to XY millimetres on the flat printed reference surface. It contains
the homography, exact marker layout, camera settings, detected corners, frame
metadata and image checks. Its reference images are in the same directory.
The public PNGs preserve the original image dimensions and calibration pixels;
everything outside the central black tabletop is black. The mask polygon and
image hashes are recorded in the calibration JSON.
The app does not automatically load this record.

The origin is the midpoint of the edge nearest the arm. +X points away from the
arm; +Y points left when viewed from the arm. The printed area covers
X = 0–360 mm and Y = −140–140 mm. Image left/right does not define these axes.

## Camera settings

| Setting | Value |
| --- | --- |
| Delivered RGB | 1280 × 720, 5 fps |
| Resize mode | Crop |
| Undistortion | Enabled |
| JPEG quality | 85 |
| Focus | Manual, lens position 65 |
| Exposure | Manual, 30000 µs |
| ISO | 430 |
| White balance | Manual, 4000 K |
| Anti-banding | 50 Hz; affects auto exposure only |

Exposure, ISO and white balance may be adjusted for lighting, provided corners
remain sharp and unclipped. Record the observed values. Preserve focus and
image geometry; changed focus or geometry requires a fresh mapping and checks.
Camera movement always requires a fresh mapping. After reconnecting, check
settings again because defaults may be restored.

## Manual recalibration

1. Keep servo power off. Secure the camera in its final position and keep the
   printed grid flat and fixed relative to the arm. Check the printed scale
   with a ruler, including both panels and their seam.
2. Apply the settings above in the Camera panel. Check `/api/camera/status`:
   settings must no longer be pending, and observed frame values must match.
   Inspect sharpness, glare and visibility of all six markers.
3. Capture five fresh stationary images through `/api/camera/capture`, retaining
   each base64-decoded `rgb_jpeg` and its `metadata` without depth fields. Use
   `after_run_id`, `after_sequence` and `wait_ms=1000` to request advancing
   frames. The Camera panel's Capture and Download RGB controls also provide
   images; retain matching metadata separately. Do not start another camera
   process while the app owns the camera.
4. Pair image points `(u, v)` with known table points `(X, Y)` in millimetres.
   Prefer all 24 corners of ArUco IDs 0–5, dictionary `DICT_4X4_50`, using
   `table_layout.markers` in the saved record. Match each detected corner to
   its printed marker corner order, never image-relative top-left order.
   Alternatively, manually select at least four widely separated, unambiguous
   grid intersections in an image viewer that reports original-image pixels.
   Avoid collinear points; use more points spread across the area when possible.
5. Fit `H_image_to_table` with OpenCV
   `findHomography(pixel_points, table_xy_points, method=0)`. This is a direct
   pixel-to-millimetre fit; camera intrinsics and depth are not required.
   Fit each frame separately to check stability, then pool corresponding points
   across the frames for the saved mapping. Use the already-undistorted delivered
   images without applying raw sensor distortion again.
6. Check several other printed grid intersections that were excluded from fitting,
   including points near the edges. Record signed X/Y errors and lateral error
   `sqrt(dx² + dy²)` in mm. Inspect marker overlays and check coordinate signs.
   For marker fitting, also refit while leaving each marker out in turn and
   check its omitted corners. Compare mappings across the stationary frames.
7. Save the new matrices, layout, settings, metadata, images, method/version and
   results together. Update the selected record only after reviewing the checks;
   retain the earlier record and images in a dated local evidence directory.
   Choose an acceptable error for the intended task before claiming acceptance.
8. For public reference images, mask the surrounding room after fitting. Select
   a polygon inside the black tabletop boundary that retains the printed grid
   and all marker corners. Set exterior pixels to RGB `(0, 0, 0)` and save PNGs
   without resizing, warping or altering interior pixels. Record the polygon
   and hashes, check the result visually, and verify interior pixels against
   the decoded source images. Select a new polygon if framing changes.

The app has no built-in homography fitter. Step 5 is an offline analysis using
OpenCV; the recorded analysis used OpenCV 5.0.0. Keep analysis dependencies
separate from the robot runtime and project dependency locks.

## Applying the mapping

For an original delivered-image pixel `(u, v)`, compute:

```text
[a, b, c] = H_image_to_table × [u, v, 1]
X = a / c
Y = b / c
```

`H_table_to_image` provides the inverse mapping. Convert preview clicks back
to original 1280 × 720 pixel coordinates before applying the matrix. Use the
mapping only while camera position, image geometry and reference layout match.
It applies to points on the printed surface; raised object centres or tops
introduce perspective offsets.

## Recorded checks

The October 11, 2026 fit at 15:44 UTC used six markers across five advancing
frames of the rearranged setup. Marker fit RMS was 0.48–0.54 mm; the maximum
error when leaving a marker out was 1.66 mm. Six separate printed grid
intersections had 0.64 mm RMS and 1.06 mm maximum lateral error. Maximum
frame-to-frame axis ranges were 0.178 mm in X and 0.076 mm in Y over the
sampled area.

Grid checks used manual image selection with local corner refinement. They
measure agreement with printed coordinates, not independently surveyed physical
accuracy.
