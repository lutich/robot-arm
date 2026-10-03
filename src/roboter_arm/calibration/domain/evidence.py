"""Calibration evidence rules: bounds, observed direction and physical zero reference."""
from roboter_arm.shared.joints import JOINTS


def validate(*, channel, joint, low, high, kind, provenance,
             direction=None, zero_count=None, zero_reference=None):
    if type(channel) is not int or channel not in JOINTS:
        raise ValueError('Channel must be 0–5')
    if kind not in ('observation', 'validated_limits'):
        raise ValueError('Kind must be observation or validated_limits')
    if not all(isinstance(v, str) and v.strip() for v in (joint, provenance)):
        raise ValueError('Observed joint and provenance are required')
    _, inherited_low, inherited_high = JOINTS[channel]
    if type(low) is not int or type(high) is not int or not inherited_low <= low <= high <= inherited_high:
        raise ValueError('Pulse bounds must be integer counts inside the historical reference range')
    if direction is not None and (not isinstance(direction, str) or not direction.strip()):
        raise ValueError('Direction must describe movement as counts increase')
    if (zero_count is None) != (zero_reference is None):
        raise ValueError('Zero count and physical reference must be supplied together')
    if zero_count is not None:
        if type(zero_count) is not int or not low <= zero_count <= high:
            raise ValueError('Zero count must lie inside recorded bounds')
        if not isinstance(zero_reference, str) or not zero_reference.strip():
            raise ValueError('Physical zero reference is required')
    if kind == 'validated_limits' and (direction is None or zero_count is None):
        raise ValueError('Validated limits require observed direction and physical zero reference')
