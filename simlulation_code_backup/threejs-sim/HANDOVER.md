# Handover, 2026-10-12

Start here in a new session. Everything below is on branch `docs/pick-and-place-plan` (local only, **not pushed**).

## 1. Goal

A 6DOF desktop arm (Tatiy kit, six MG996R servos, `docs/amazon-product-info.md`) should **move and stack coloured 26,5 mm cubes on request**, for example "put blue on orange".

- The arm is driven by a Raspberry Pi with a PCA9685 servo board. The controller software and its web UI are a separate repo, `lutich/robot-arm`. On the team network the controller runs at http://10.102.4.103:8765/.
- Servo values are **PCA9685 counts at 60 Hz**, not degrees. The servos give no position feedback.
- Plan: **open loop**. Start from a calibrated HOME pose, compute joint angles with inverse kinematics from measured link lengths, and convert them to counts with the measured counts per degree.
- An **RGB camera** (OAK, at the far end of the mat) finds the cubes. It could later make fine corrections near the cube. **Its depth output is unusable**, so we use RGB only.
- A **simulator** to develop and check all this without the real arm: the three.js simulator in `threejs-sim/`, see section 5.

## 2. Repos and files

| Where | What |
|-------|------|
| `C:\Users\adm_bjung\Documents\px-robotic-arm-sim` | This repo (GitHub: `MelJan/px-robotoc-arm-sim`, note the "robotoc" spelling) |
| `C:\Users\adm_bjung\Documents\robot-arm` | Clone of `lutich/robot-arm`, the controller. Reference only; nothing there was changed. `vision/` reads its mat layout from `config/camera-table-xy/calibration.json` |

| File | Content |
|------|---------|
| `docs/arm-dimensions.md` | Link lengths, gripper, board, coordinate frame (pre-existing, small edits) |
| `docs/servo-calibration.md` | HOME, PARK, count guards, counts per degree, gripper direction |
| `docs/pick-and-place-plan.md` | The open-loop plan: IK formulas, motion sequence, reach, error budget, open items, simulation results |
| `docs/tracking-plan.md` | Camera: finding cubes and stack levels with RGB only, arm tracking ideas, stacking flow, reach per stack level |
| `docs/reference-robot-arm-repo.md` | Facts from the controller repo: units, guards, HTTP API, camera calibration and its frame |
| `docs/controller-live-2026-10-12.md` | What the live controller showed: camera frame, UI, live HOME, the team's hand-taught pick and place |
| `docs/simulator-proposal.md` | The pasted proposal: Godot vs Unreal + URLab vs MuJoCo |
| `docs/unreal-setup.md` | Unreal 5.8 + MCP setup, toolchain, URLab version conflict. **Unreal route dropped on 2026-10-12; `unreal/` and `.mcp.json` were deleted** |
| `vision/blocks.py` | Cube finder: camera fit from the mat markers, colour segmentation, stack levels, positions |
| `threejs-sim/` | **The simulator**: browser app with kinematics, controller emulation, collision, cube physics, planning, camera calibration and cube recognition. `threejs-sim/README.md`, design in `docs/threejs-sim.md` |
| `integration/` | Patch for `lutich/robot-arm` that serves the simulator under `/sim/` (not applied) |
| `docs/images/photos/` | All real photos of the arm and setup (renamed from `Media*.jpg` and the old gripper/arm photos) |
| `docs/images/web/` | Kit photos and bracket/servo/horn drawings from the web, with sources and dimensions in `SOURCES.md` (third-party, internal reference) |

## 3. What works (checked)

| Item | Result |
|------|--------|
| Simulator tests (`threejs-sim/test/`) | 18 tests: IK against the plan's table, FK round trip, gripper linkage, controller rules, collision, cube physics (stable stack, topple, push), pick/place and two-level stacking, camera calibration against OpenCV |
| Camera model (`vision/blocks.py`) | Fitted from the mat markers in every frame: 0,86 px error, f = 952 px, camera 37 cm above the mat, 12 cm beyond the far edge. The stored homography from 2026-10-11 is outdated (off by up to 39 mm) |
| Cube on the real mat | Found at X = 154,5, Y = -108 mm, level 0, front edge 26,1 mm (real 26,5) |
| Stacked cubes, synthetic | Orange, blue on orange, yellow on blue, green alone: all levels right, 0,6 to 2,1 mm error (checked with a MuJoCo render that has since been removed) |
| Unreal MCP | UE 5.8.3 started headless with `unreal/PxArmSim`; the MCP server answered and listed 52 toolsets, including actor, scene, mesh and viewport-capture tools |

Commands:

```powershell
cd C:\Users\adm_bjung\Documents\px-robotic-arm-sim
node threejs-sim\server.mjs --mock                                                  # simulator, http://127.0.0.1:8770/
node --test threejs-sim\test\*.test.mjs
.\.venv\Scripts\python.exe vision\blocks.py docs\images\photos\controller-camera-2026-10-12.jpg --out annotated.png
.\.venv\Scripts\python.exe vision\blocks.py --url http://10.102.4.103:8765 --every 5     # GET only, never moves the arm
```

## 4. Open items, most important first

1. **Joint angle at HOME and direction for each servo.** Without these, IK angles cannot become counts. Best route: a small ArUco marker on the gripper, then fit from a few commanded poses (`docs/tracking-plan.md`, section 4).
2. **Which HOME.** The live controller has S5 = 250, S6 = 550; the docs and controller repo have S5 = 480, S6 = 650.
3. **Offset `d`** from the S1 axis to the mat's near edge. It links the camera frame to the arm frame. From the overhead photo `docs/images/photos/setup-top-view-base.jpg` about −2,5 cm (S1 is 2,5 cm *in front of* the mat's near edge; the mat overhangs the board's back edge). Measure it, or fit it from an S1 sweep.
4. **Colours.** Only orange is tuned. Take one frame with all real cube colours on the mat.
5. **A real stack frame** to check level detection outside the simulation.
6. **Reach.** A stack's third cube needs the stack within about 16 cm of S1 (2 cm hover). Close positions need S2 to lean backwards. S4 has only about 47° below HOME before its guard.
7. **Counts per degree** come from one 90° move per joint; S1 was read as 85°.

Already answered: the gripper opens at 490, grips the cube at 590 and is fully closed at 650. The tick unit is PCA9685 counts at 60 Hz.

## 5. Simulator

**Decided and built: `threejs-sim/`.** Only three.js and our own code: the MuJoCo scripts (`sim/`, `vision/test_synthetic.py`) were removed on 2026-10-12, their IK and checks live on in the simulator and its tests. Run `node threejs-sim/server.mjs --mock` and open http://127.0.0.1:8770/. 18 tests: `node --test threejs-sim/test/*.test.mjs`.
What it covers, how, and the next steps (including the controller-page integration) are in `docs/threejs-sim.md`.
The rest of this section is the earlier state of the Unreal route, kept for reference. **The Unreal project (`unreal/`), the cloned URLab plugin and `.mcp.json` were deleted on 2026-10-12 at the user's request.** Unreal Engine 5.8 itself is still installed.

State of the Unreal route when it was stopped:

- Unreal Engine 5.8.3 is installed and its MCP works with `unreal/PxArmSim`.
- The user also opened `unreal/PxArmSim 5.8/`. It looks like a copy the launcher made. Its `.uproject` has a machine-specific engine GUID; it is **not committed**.
- URLab (MuJoCo inside Unreal) is **cloned but not built** in `unreal/PxArmSim/Plugins/UnrealRoboticsLab` (commit `efbc5e2`, not committed here). It officially supports **UE 5.7 only**. Building it needs a C++ project (Visual Studio and CMake are installed).
- Nothing in the actual task needs Unreal so far. The kinematics, physics check and vision all run in Python with MuJoCo.

**Recommendation: drop Unreal for now and build a small web app.**

- **Viewer:** three.js, one HTML page.
  - Draws the arm from `docs/arm-dimensions.md` (four links and a gripper are simple boxes and cylinders).
  - Shows the current pose from the controller's commanded counts.
  - Shows the cubes found by `vision/blocks.py`.
  - Previews a planned motion before it is sent.
- **Backend:** a small Python server (FastAPI, like the controller), so the existing code is reused as is:
  - wraps `sim/ik.py`, `vision/blocks.py` and the MuJoCo check;
  - forwards to the controller API.
  - The controller only accepts POSTs from its own origin (`Origin: http://<host>`), so a page from elsewhere cannot command the arm directly. A proxy can send the matching header.
- **Optional:** MuJoCo also runs in the browser through community WebAssembly builds, if physics is wanted in the page. That is not needed for kinematics.

Why: a few hundred lines instead of a C++ Unreal build. It also runs on any laptop next to the arm, and it reuses everything that already works.
Unreal (or URLab) only pays off if photoreal rendering or synthetic camera images for training become a goal.

## 6. Environment on this machine (set up 2026-10-12)

| Tool | State |
|------|-------|
| Python venv | `.venv` (Python 3.13) with `numpy` and `opencv-python` 5.0, only for `vision/blocks.py`. Recreate with `py -3.13 -m venv .venv` and `pip install numpy opencv-python`. MuJoCo was removed on 2026-10-12 |
| Unreal Engine | 5.8.3, `C:\Program Files\Epic Games\UE_5.8` |
| Visual Studio 2022 Community | 17.14.41, MSVC 14.44 (cl 19.44.35229), installed via winget. **Licence:** Community may not be allowed for a company of this size; check whether Professional is needed |
| CMake 4.4, Node.js 24 | User scope, via winget |

## 7. Safety rules used so far

- Only GET requests to the controller: `state`, `camera/status`, `camera/snapshot.jpg`, `camera/capture`, `distance/status`, `openapi.json`. No POST was ever sent. The arm was powered and armed while it was read.
- `demo/load` was not used, because it replaces the demo draft the team is working on.
- Anything that moves the arm should go through the controller's own guards, at low demo speed, with someone at the emergency stop.
- **Rule from the user (2026-10-12): do not command the real arm, and keep even reads to a minimum (other people use it; the camera and servos can overheat).** The simulator is read only towards a real controller unless its server is started with `--allow-motion`, and camera frames are fetched one at a time on request. Test against `server.mjs --mock`.

## 8. Git

```
a87fa26 Document live controller state and camera view
4cc08f1 Add MuJoCo model, IK check, controller reference and Unreal project
7e707a8 Add Unreal MCP setup doc and project MCP config
83291ec Add pick-and-place plan and simulator proposal docs
```

The commit with the tracking plan, `vision/` and this handover follows these.
The Unreal folders were deleted when the route was dropped.
