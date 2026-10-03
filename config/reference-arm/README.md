# Reference arm observations

Selected evidence from the owner's existing Raspberry Pi arm, recorded on
2026-10-01. Private machine paths and references to unpublished logs have been
removed from the public copies; counts, units, observation scope and validation
status are retained.

| File | Meaning |
| --- | --- |
| `inherited.json` | Historical channel settings from recovered `roboarm/config.json` |
| `gripper-observation.json` | Narrow attended channel-5 check only |
| `park.json` | Retrospective observed supported full PARK commands |
| `home.json` | Observed HOME saved through the commissioning app |

These files are not measured position or general motion validation. They are
not automatically loaded as app configuration. See [calibration](../../docs/calibration.md).
Deployment refuses to overwrite differing reference JSON, even with a baseline.
