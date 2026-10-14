# Three.js implementation

Status: calibration-focused simulation baseline, 2026-10-13.

## Runtime boundary

The browser starts in simulation and performs no robot request on page load.
The physical controller is reached only through the same-origin gateway; the
browser never accesses GPIO, PCA9685 or serial hardware directly.

Current mode behavior:

| Mode | Allowed motion |
| --- | --- |
| Simulation | reference poses, joint sliders, IK/task playback |
| Connected read-only | none |
| Operational disarmed | none |
| Operational armed | explicit one-joint slider commands only |
| Fault stopped | none |

Automated physical reference-pose and task playback is disabled while no
calibrated collision model is active. Live Mode still requires explicit
arming, supported-PARK confirmation, backend bounds, command-rate limiting,
single-joint command shape, API confirmation and emergency stop.

## Architecture

```text
index.html / style.css             responsive dark application shell
js/app.js                          UI, scene, camera and mode orchestration
js/model/                          detailed photo-informed Three.js geometry
js/kinematics.js                   preserved visual kinematics (centimetres)
js/advanced.js                     visual/canonical calibration adapter
js/planner.js                      task waypoints and physics-outcome playback
js/world.js                        rule-based grasp plus cube dynamics
js/arm-physics-shapes.js           simulated arm/cube interaction geometry
src/config/spec.js                 canonical specification validation
src/core/kinematics.js             millimetre FK and numerical IK
src/core/planning.js               direct calibration planning and cost metrics
src/core/controller.js             virtual interpolation/playback
src/core/physics.js                lazy Rapier3D adapter
src/operational/client.js          guarded robot API state machine
src/operational/gateway.js         proxy allowlist
server.mjs                         static server and same-origin gateway
public/arm-spec.json               canonical millimetre runtime data
test/*.test.js                     Node regression suite
```

Removed during the calibration refactor:

- analytical capsule/box collision module;
- detailed-geometry contact policy;
- collision-aware transit insertion and RRT* clearance search;
- runtime contact-stop policy and collision overlay; and
- collision-specific tests and UI controls.

## Units and transforms

Canonical kinematics and planning use millimetres. Rapier receives metres at
its adapter boundary. The detailed renderer uses centimetres and is converted
by `js/advanced.js` so its existing visual calibration remains unchanged.

- origin: board top below S1;
- +Z: up;
- +Y: back edge toward the front of the board;
- +X: across the 300 mm board width.

Transform chain:

```text
board -> S1 Z -> 96 mm -> S2 -X -> 105 mm
      -> S3 -X -> 126 mm -> S4 -X -> 67 mm
      -> S5 Z -> S6 tool length -> TCP
```

S2–S4 may use provisional photo anchors. S6 linearly approximates the linkage
between counts 490 and 650. Orientation IK is not claimed.

## Specification and provenance

`arm-spec.json` schema version 2 separates:

- measured dimensions;
- provisional visual/calibration assumptions;
- observed commissioning HOME/PARK counts;
- historical unvalidated bounds; and
- validated safe limits (currently empty).

```text
HOME: 320, 427, 403, 220, 480, 650
PARK: 330, 540, 580, 195, 480, 650
```

PARK is an observed supported pose on the provisional 25 mm raised fixture
surface. It is not declared a validated transition or safe unattended pose.
`world.physics.robotRadii` exists only to let the simulated arm push cubes; it
is not clearance or safety geometry.

## Planning and simulation

Reference poses use direct joint-space interpolation in Simulation mode. Text
tasks expand to:

```text
approach pick -> pick -> grasp -> lift
-> approach place -> place -> release -> retreat
```

Each Cartesian waypoint is solved by numerical IK from the prior count vector.
Historical count bounds are checked. The result is explicitly tagged
`unchecked-calibration`; there is no clearance test or shortest collision-free
path claim.

Two virtual playback modes remain:

- coordinated interpolation for visual simulation;
- deterministic S1→S6 one-servo steps for controller-style preview.

The reported motion cost covers changed servos, count travel, estimated
duration, acceleration proxy and TCP distance. It does not contain a clearance
term.

Rapier simulates gravity, friction, cube/board/fixture interaction and arm/cube
impulses. The gripped cube follows the modeled tool until release. Physics is
used to observe object outcomes; it neither accepts nor stops an arm path.

## Camera and calibration

Mat markers 0–5 calibrate the camera. Marker 6 is reserved for optional
gripper/TCP observation. Color segmentation reports candidate cubes in the
board frame; Z assumes a cube resting on the planar mat unless depth evidence
is available.

The calibration map displays recorded Pos1–Pos3 left/right coordinates,
nearest fiducial, current marker pixels and stored S1–S6 counts. These are
calibration observations, not safe limits.

## Operational API

Gateway policy:

| Startup | Forwarded controller operations |
| --- | --- |
| `npm run serve` | none |
| `npm run serve -- --controller URL` | state/camera GET, emergency stop |
| add `--allow-motion` | power and single-joint move routes |

The browser labels controller counts **commanded pose** because the servos have
no position feedback. In armed calibration mode, a slider sends only on
release. The client validates the target against current backend bounds and
waits for confirmation. A request failure attempts `/api/stop` and enters the
fault-stopped state.

## Verification

`npm test` currently runs 39 tests covering schema/provenance, FK/IK,
calibration planning, deterministic one-joint conversion, Rapier initialization,
camera/marker tracking, calibration mapping, gateway allowlists, operational
state validation and command failure handling.

`npm run build` succeeds. Vite reports only non-fatal large-chunk warnings for
Three.js and the embedded Rapier WebAssembly runtime.

Manual checks after UI changes:

1. Console has no load/runtime error.
2. HOME/PARK/STRAIGHT UP differ in Simulation mode.
3. Plan result says collision checks are disabled.
4. Run in sim animates the arm and updates cube outcome.
5. Real playback remains disabled.
6. Armed joint sliders still require explicit connection/arming and one-joint
   confirmation.

## Reintroducing collision planning

Do not re-enable motion gating from photo-derived envelopes. First survey link,
claw, cable, fixture and board geometry; validate joint zeros/directions; define
real safe limits; create measured swept-volume fixtures; and verify HOME↔PARK
plus task trajectories against independent observations. Only then add a new
collision planner behind tests and an explicit operational review.
