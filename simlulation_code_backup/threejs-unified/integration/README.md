# Integration with the controller (lutich/robot-arm)

`robot-arm-sim-route.patch` lets the controller serve the three.js simulator from this repo under `/sim/`, and shows a "3D simulator" button in its page header.
It is **not applied** to the controller repo. That repo belongs to the team, and the patch needs their review first.

| File | Change |
|------|--------|
| `src/roboter_arm/control/presentation/http_api.py` | Read-only `GET /sim/` and `/sim/{path}` serving `ROBOT_ARM_SIM_DIR` (allowed file types only, no path escape). Only active when the variable is set |
| `web/manual_control.html`, `.css`, `.js` | "3D simulator" link in the header, shown only when `/sim/index.html` answers |

Checked: the patch applies to commit `3792d64`, and the changed Python file compiles.
Not checked: running it on the Pi.

## Apply (after the team agrees)

```sh
cd robot-arm
git apply ../px-robotic-arm-sim/integration/robot-arm-sim-route.patch
```

On the Pi, copy `threejs-sim/` somewhere readable and set the variable in the service environment, for example:

```sh
ROBOT_ARM_SIM_DIR=/home/pi/threejs-sim
```

Then open `http://<pi>:8765/` and click "3D simulator".
The simulator is then same-origin with the API.
It reads `/api/state` after "Connect", and it moves the arm only after the operator ticks the consent box. Every move first passes the simulation's collision check and then the controller's own guards.
