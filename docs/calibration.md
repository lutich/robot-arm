# Calibration and reference data

`config/reference-arm/` retains the historical guards and selected physical
observations used by the imported application. It is reference evidence for
one arm. There are no measured general joint limits, angle zeros, geometry or
validated XYZ model supplied here.

The app currently defines channel guards in `src/roboter_arm/shared/joints.py`
and PARK/HOME commands in `src/roboter_arm/control/application/session.py`. Reference JSON documents their
provenance; it is not loaded as a user configuration. Adapting another arm
requires explicit reviewed settings and physical commissioning, not just
copying this arm's poses.

Ordinary position servos in this setup do not return actual position through
PCA9685. Store commands as commands. Record units, nominal PWM frequency,
channel assignment, observation date and what was physically checked.

The calibration tool appends UUID-named records without hardware access:

```sh
uv run --locked python scripts/calibration.py --help
```

An `observation` can record a limited checked envelope. `validated_limits`
additionally requires physical direction and a zero count/reference. Do not
label synthetic data as physical validation. Records remain within the inherited
outer guards; extending those guards is a separate reviewed change.

`load_limits` (`src/roboter_arm/control/infrastructure/evidence_limits.py`) and
`Control` (`src/roboter_arm/control/application/control.py`) accept explicitly
selected validated records and speeds for general control primitives. They are
an offline evidence-gated component, not the commissioning app's lifecycle
owner or a second running controller.
`Session` continues to own the current app's attended commissioning behavior.

Generated calibration is under ignored `artifacts/calibration/`; poses and
demos are separated into `synthetic` and `commissioning` scopes. Deployment
preserves these directories, including records from the previous installation.
It does not copy laptop recordings to hardware or choose the latest record
implicitly.

For later coordinate work, measure joint axes/link lengths, define the table
frame and tool point, record observed XYZ alongside commands, and keep fitting
samples separate from validation samples. Coordinate control is not implemented
in this starter.
