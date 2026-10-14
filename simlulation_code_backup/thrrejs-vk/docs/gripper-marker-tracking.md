# Gripper marker tracking

This workflow estimates the physical tool-centre point (TCP) from a camera
frame without commanding the robot. The six printed mat markers calibrate the
camera; a separate OpenCV `DICT_4X4_50` marker, ID 6, identifies the gripper.

## Hardware setup

1. Download `web/public/gripper-marker-id6.svg` from the Camera tab.
2. Print at 100% scale. The black marker square must measure **30.0 mm**; the
   surrounding 5 mm white quiet zone is not part of that measurement.
3. Mount it rigidly on the gripper where the complete border stays visible.
   Do not obstruct the jaws, linkage or cables.
4. Measure the marker-centre-to-TCP vector in millimetres. Marker coordinates
   are +X right, +Y toward the printed top and +Z out of the printed face.
5. Add the marker and its mount to the surveyed collision envelope before
   treating collision results as representative of the modified arm.

The specification deliberately starts with `[0, 0, 0]` and status
`unattached-and-offset-unmeasured`. Until the measured offset is entered, the
result is the marker centre—not the physical TCP.

## Use with a photograph

1. Open **Camera** and select **Image file…**.
2. Select a frame containing at least two mat markers and gripper marker ID 6.
3. Run **Find markers & calibrate**.
4. Enter the measured marker size and marker-to-TCP offset.
5. Select **Find marker & compare**.

The overlay shows planned, simulated and observed TCP points. The readout
reports their coordinates and Euclidean differences in millimetres.

## Use with the controller camera

Build the site, then start the read-only gateway:

```sh
cd web
npm run build
npm run serve -- --controller http://ROBOT_HOST:8765
```

Select **Connected camera snapshot**. Refresh is explicitly requested; merely
opening the simulator does not contact the controller. Continuous tracking is
also explicit and connected snapshots are limited to one request per second or
slower. The gateway exposes camera status/snapshot GET routes but not the MJPEG
stream, and camera tracking never enables motion.

## Interpretation and limitations

- Planned TCP is the active target waypoint (or current pose while idle).
- Simulated TCP is the current digital-twin pose.
- Observed TCP is the camera estimate after applying the marker-to-TCP offset.
- A single square marker provides a 6-DoF pose only while all four corners are
  clear. Glare, motion blur, flex and an incorrect printed size bias the result.
- This is metrology evidence, not joint feedback and not a safety interlock.
- No real-arm motion is validated by a successful marker detection.
