# Proposal: target architecture for the simulator

Status: **proposal**, not decided yet.
Saved from a team discussion on 2026-10-12.
The tool versions and features below come from that discussion and have not been checked yet.

The key question is whether we build a **visual robot simulator** or an **accurate robotics physics simulator**.

| Option | Strength |
|--------|----------|
| Godot | Interactive 3D app with sliders for each joint, XYZ targets, camera controls and a clean UI |
| Unreal Engine | Photorealistic graphics, complex environments, advanced virtual cameras |
| MuJoCo | Accurate robot kinematics, dynamics, motor forces and manipulation experiments |

Blender should not be the main simulator.
It stays useful for building the robot's 3D meshes (see the Blender setup in `arm-dimensions.md`).

## 1. Unreal + MuJoCo: Unreal Robotics Lab (URLab)

Repository: https://github.com/URLab-Sim/UnrealRoboticsLab

URLab is an open-source project that integrates MuJoCo physics directly into Unreal Engine 5.
It can import MJCF robot models, control actuators through Python or Blueprints, and visualise sensor data.
With it we would not have to choose between realistic physics and good graphics.

Downside: the setup is complex.
The repository currently asks for Unreal Engine 5.7, Visual Studio with the C++ build tools, and a separate Python bridge.

Related example: the AMD ROCm blog post "Utilizing AMD Schola and UnrealRoboticsLab with AMD ROCm Software to Train a Robotic Arm".

## 2. Comparison for our arm

| Capability | Godot 4 + Jolt | Unreal + URLab | MuJoCo standalone |
|------------|----------------|----------------|-------------------|
| 3D appearance | Good | Excellent | Basic |
| Joint manipulation | Yes | Yes | Yes |
| Inverse kinematics | Implement it or use a library | MuJoCo-based | MuJoCo-based |
| Servo torque modelling | Custom | MuJoCo | MuJoCo |
| Object grasping | Possible | Yes | Yes |
| Python integration | External bridge | Supported bridge | Native |
| Ease of initial setup | Easy | Difficult | Easy |
| Custom UI | Excellent | Excellent | More work |

Godot supports articulated physics through joints and the Jolt physics backend.
Community ROS 2 integrations exist, but they are less mature and less integrated than the dedicated MuJoCo solutions.

Unity + MuJoCo is another option: the MuJoCo repository maintains a Unity plugin.

## 3. Proposed architecture

```
Python robotics controller   inverse kinematics, motion planning, servo control
        |
MuJoCo physics engine        6 joints, gravity, torque, contact physics
        |
Unreal / URLab               3D viewer, lights, camera, interactive controls
        |
Physical robot               servo commands, feedback where available
```

Build the arm's mechanical structure in **MJCF** first, with the real joint-axis positions from `arm-dimensions.md`.
That keeps the physics model independent of the renderer, so we can replace or upgrade the renderer later.

The Python controller is where the open-loop pick-and-place logic from `pick-and-place-plan.md` would live.
The same code could drive the simulation and the real arm.

## 4. Recommendation

| If the priority is | Use |
|--------------------|-----|
| Visual quality | Unreal Engine + URLab + Python |
| A lightweight interactive desktop app for moving the arm | Godot |
| Precise kinematics working with the least effort | MuJoCo directly |

## 5. Open points

- Decide which priority matters most for us: visuals, an interactive app, or accurate kinematics.
- Check the URLab requirements (Unreal Engine version, build tools) against our machines.
- Check whether the MG996R servo data in `amazon-product-info.md` (torque, speed) is enough for a MuJoCo actuator model.
