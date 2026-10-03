# Attended arm operation

Read [hardware](hardware.md) first. The shipped app uses this arm's historical
count guards and observed PARK/HOME commands. Neither those guards nor the
coordinated transitions have general physical validation. For another arm,
commission its own mapping, supported starting condition and limits before
using these commands. Editing reference JSON alone does not reconfigure the app.

## Startup

1. Keep servo power off. Secure wiring and common ground; establish support
   against loss of holding torque. Do not force joints by hand.
2. Establish the active PARK definition's physical supported pose. Clear the complete
   PARK-to-HOME path, check supply readiness and keep the manual supply switch
   reachable. Saved counts do not establish current physical position.
3. Open the Pi app at http://raspberrypi.local:8765/. The service starts it at boot
   (`scripts/service.py start` otherwise). Launch has no controller access; after a
   (re)start, state begins unprepared/unarmed with unknown commands.
4. With the operator attending and the supply set for the agreed test, turn
   servo power on manually. Check the UI's supported PARK/readiness box and
   click Power on only when ready for that exact path.

Power on explicitly initializes the controller, clears/checks outputs, enables
six PARK targets, then moves to HOME. The first targets can move a joint if the
physical starting pose differs. Later interpolation cannot constrain that
unknown first displacement.

| Shipped default pose | Channel 0–5 commands, nominal 60 Hz |
| --- | --- |
| PARK | 330 / 540 / 580 / 195 / 480 / 650 |
| HOME | 320 / 427 / 403 / 220 / 480 / 650 |

These poses were observed individually on 2026-10-01. The full coordinated
paths and full-arm supply capacity remain unvalidated. The reference diagrams
are illustrations, not position measurements.

## Manual movement and Demo playback

Select a joint using the labels. Slider release or Move submits an integer
target; arrows add/subtract Step size. Other joints hold their last commands.
The shoulder and wrist-flexion arrows retain their count mapping. The elbow's
up arrow decreases counts and its down arrow increases counts, matching the
owner's reported direction. The API's jog direction remains a count sign;
the UI shows the signed count step in its movement notice.
Base rotation and wrist roll share simple clockwise/counterclockwise symbols.
Clockwise adds counts and counterclockwise subtracts counts, following the
owner's direction mapping. The reference views are from above for the base
and from the gripper along the wrist toward the arm for wrist roll. The
gripper uses a jaw symbol with the − button above the + button; their labels
and commands both show the count sign.
Only one movement owns the controller at a time.

Manual moves and pose transitions use a continuous command curve bounded at
20 counts/s before integer rounding. This is not measured mechanical velocity.
Historical ranges are commissioning guards, not collision avoidance.

Demo capture requires all six settled commands. Next plays one position; Run
plays the sequence once; Pause finishes the current step and holds; Resume
continues. Demo speed is 1–20 counts/s. Each segment needs observed clearance;
valid endpoints alone do not validate the path. Save writes a new immutable
record; Load does not move or restore execution progress.

Min/Max and Step size/Demo speed changes last only for the running session.
Save HOME/PARK archives observed counts and provenance and makes the saved pose
active immediately, without moving the arm. Go HOME/PARK and Power on/off use
these definitions, including after restart. Preview and hardware definitions
are stored separately under `artifacts/poses/`; deployment preserves them.
Before each power transition, check the active values displayed in the app and
the physical starting pose, support and clearance for that path.

After upgrading from the archive-only behavior, re-save the desired HOME and
PARK poses. Existing archival records are retained but are not activated
automatically. Reference JSON remains historical evidence.

Stop the test for unexpected movement, binding, sustained buzzing, current
limiting or voltage drop. Use the physical servo supply switch if needed;
software cannot guarantee bus cleanup. Keep the arm supported when holding
torque is removed. Do not automatically raise current limits or expand travel.

## Normal shutdown and immediate stop

With the return path clear and support maintained, Power off moves from known
commands to PARK, disables all PWM outputs and checks readback. A failed return
does not establish physical PARK. Turn the servo supply off manually afterward.
The app keeps running for the next session; `scripts/service.py stop` shuts it down.

Emergency stop cancels and directly disables PWM without a PARK movement.
Driver faults, cancellation and handled exits attempt the same direct stop.
The physical switch remains necessary for a stuck bus, unhandled process kill
or loss of connectivity. Disabling PWM does not cut supply power.

Closing, hiding or losing the browser does not stop the arm. It keeps holding
torque, and a running move or Demo continues to its end. Use Stop or Power off
before leaving the page; both work from any device on the Pi's network. The app
runs as a service, so closing the laptop or losing the network leaves it running
and holding until someone uses Stop or Power off, `scripts/service.py stop` stops
it, or servo power is switched off. Ten minutes without accepted
movement/pose save produces a warning; it does not release holding torque. Another Power on requires fresh
physical PARK/readiness confirmation.

The controller lock prevents concurrent use by the imported tools. Retaining
`artifacts/controller.lock` also preserves the lock path used by the prior
installation. Run one deployed app at a time; do not launch recovered legacy
applications, which can command movement at construction.

## Commissioning tools

Preview a single-joint check without hardware:

```sh
uv run --locked python scripts/joint_check.py --channel 4 --start 480
```

`--prepare` and `--execute` are hardware actions. The tool requires explicit
support/power/readiness flags and a bounded selected-joint test. Prepare a
specific target envelope, duration and stop arrangement with the operator
before using them; setup scripts do not grant powered readiness.
