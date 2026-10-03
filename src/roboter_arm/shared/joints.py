"""Joint catalog shared by control and calibration: channel -> (name, low, high)."""

# Historical counts at nominal 60 Hz, not calibrated limits or current pose.
JOINTS = {
    0: ('S1 · base rotation', 90, 660),
    1: ('S2 · shoulder pitch', 185, 660),
    2: ('S3 · elbow pitch', 90, 660),
    3: ('wrist flexion', 90, 660), 4: ('wrist rotation', 90, 660),
    5: ('gripper', 490, 660),
}
