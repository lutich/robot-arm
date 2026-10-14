# 3D simulation

The manual control page has a **3D simulation** section that draws a simple
six-joint arm and a few blocks on a table. It uses the state the page already
polls, so every control on the page moves the simulated arm: Power on/off, the
joint arrows, Move, Go HOME/PARK and Demo playback. Anything else that posts to
the API moves it too, such as Swagger at `/docs`, `curl` or an agent. There is
no physics engine and no separate process; the viewer is plain HTML, CSS and
JavaScript with no dependencies, so it also works offline on the Pi.

Start the preview and open **http://127.0.0.1:8765/**:

```sh
uv run --locked python scripts/manual_control.py
```

**http://127.0.0.1:8765/simulation** is the same viewer in its own window,
without controls, for a second screen. It polls the state itself and mirrors
the same session. On a deployed Pi both are served at `raspberrypi.local:8765`.
In a hardware session the viewer mirrors the counts the real arm receives; the
line under the view says so. The viewer never sends commands by itself.

## What the page shows

- A table with a 50 mm grid, the arm, and coloured blocks. Drag to orbit,
  scroll to zoom, or use the Iso/Front/Side/Top buttons.
- A HUD with the grasp point (the point between the finger tips) in millimetres,
  the gripper opening and the block being held.
- A block list with positions, a status line for pick/release events, and
  Reset blocks.

Before Power on, or whenever a command is unknown, the arm is drawn grey at the
PARK counts. The joint directions follow the page's jog arrows: base + is clockwise from
above, shoulder ↑ and wrist flexion ↑ raise the gripper, elbow ↑ raises the
forearm. The gripper opens towards lower counts in this model, so jog S6
GRIPPER − to open and + to close. Axes in the view: X right (red), Y forward (green), Z up (blue).
The base sits at the origin; the table extends in front of the arm.

## Blocks

Blocks are a browser-side toy world with simple contact rules:

- The gripper's palm and two fingers are collision boxes. A block they hit is
  pushed aside, and a pushed block shoves its neighbours instead of overlapping
  them. A gripper pressing down on a block, or reaching below the table, does
  not move anything; the HUD shows a red collision message instead.
- To pick a block up, open the gripper (S6 −), bring the block between the
  fingers so it fits in the gap, and close (S6 +). The HUD says when a block
  is between the fingers. The block is held once the fingers close onto its
  faces; they then rest on the block instead of closing through it.
- A held block keeps its position relative to the gripper, so raising the arm
  lifts it. It cannot be pushed below the table or into a block beneath it.
- Opening the gripper past the block size releases it. It settles on the table
  or on top of a block whose footprint it overlaps.
- Reset blocks returns them to their start positions.

The block positions and these rules exist only in the page. Reloading the page
resets the blocks; the backend knows nothing about them, and the separate
viewer window keeps its own blocks.

## Model assumptions

The geometry, link lengths and count-to-angle mapping are a visual model
chosen so that the reference HOME and PARK counts look plausible. They are not
measured, not calibrated and not a reachability or clearance check. The model is
the `MODEL` object at the top of `web/simulation.js`:

| Setting | Value | Meaning |
| --- | --- | --- |
| `degPerCount` | 180 / 570 | Nominal servo travel over the historical 90–660 count span |
| `reference` | reference HOME counts | Counts at which the listed joint angles apply |
| `joints[c].angle`, `.sign` | per joint | Angle at the reference count and direction of increasing counts, matching the page's jog arrows |
| `gripper.open`, `.closed` | 490, 660 | Counts assumed fully open and fully closed |
| `gripper.maxOpening` | 60 mm | Finger gap at `open` |
| `links` | mm | Shoulder height 60, upper arm 105, forearm 100, wrist 50, finger 35 |
| `fingerDepth`, `fingerThickness`, `palmWidth` | mm | Gripper collision boxes |
| `blocks` | list | Name, size, start X/Y and colour |

Edit these values to match a measured arm. The reference HOME is fixed in the
model so that saving a new HOME in the app does not change how counts map to
angles. Whether higher gripper counts close or open the real gripper has not
been established; the observation on file only confirms that channel 5 moved
between 595 and 605 counts.

## Driving the simulation from outside the browser

The API's same-origin rule applies: a POST needs `Origin` equal to
`http://<Host>` and a JSON body. From a terminal on the laptop running the
preview:

```sh
H=http://127.0.0.1:8765
curl -s -X POST $H/api/power_on -H "Origin: $H" -H 'Content-Type: application/json' -d '{"park_confirmed": true}'
curl -s -X POST $H/api/move -H "Origin: $H" -H 'Content-Type: application/json' -d '{"channel": 5, "target": 490, "first_clear": false}'
curl -s -X POST $H/api/move -H "Origin: $H" -H 'Content-Type: application/json' -d '{"channel": 1, "target": 395, "first_clear": false}'
curl -s $H/api/state | python3 -c 'import json,sys; print(json.load(sys.stdin)["commands"])'
```

Moves run at the backend's commanded speed; poll `/api/state` for `busy` to
know when one has finished. Keep the control page open to watch the result.
In the browser console `simulation.world()` returns the grasp point, the held
block and all block positions, `simulation.step(counts)` applies a pose to the
block world without the API, and `simulation.model` exposes the model.

Against a hardware session these same requests move the real arm. Follow
[operations](operations.md) and keep an attending operator with the servo
power switch in reach.
