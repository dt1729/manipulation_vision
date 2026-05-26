# manipulation_vision

A toolkit for vision-based robotic manipulation supporting MoveIt, reinforcement learning, and vision-language-action (VLA) pipelines — with sim-to-real transfer on the roadmap.

## Overview

`manipulation_vision` provides a unified framework for developing and deploying manipulation policies on the Piper robotic arm. It integrates physics simulation, classical motion planning, and modern learning-based approaches under a single codebase.

## Pipelines

### MoveIt (Classical Motion Planning)
- Joint and Cartesian space motion planning
- Collision-aware trajectory generation
- RViz-based visualization and interactive control
- CAN bus communication with the Piper arm via ROS Noetic

### Reinforcement Learning / Synthetic Data (robocasa + robosuite)
- PiperArm and PiperOmron (Piper arm on Omron mobile base) integrated into robosuite
- Kitchen environment simulations via robocasa for pick-and-place, rearrangement tasks
- Domain randomization support for sim-to-real transfer
- MuJoCo-based physics with OSC_POSE and HYBRID_MOBILE_BASE controllers
- Tactile sensing on gripper finger pads (see [Tactile Sensing](#tactile-sensing) below)

### VLA (Vision-Language-Action)
- Designed to support VLA model inference for manipulation tasks
- Vision input pipeline for perception-driven control
- Language-conditioned policy execution

## Tactile Sensing

The Piper gripper finger pads (link7 and link8) are instrumented with a 16×16 grid of
independent MuJoCo `<touch>` sensors per finger — 512 sensors total.

### Why not `touch_grid`?

MuJoCo's `touch_grid` plugin is the natural first choice but has a fundamental limitation:
it fires **one cell per contact point**, and MuJoCo's CCD contact model generates at most
one contact point per geom pair. With a rigid object like an orange (16 convex collision
geoms), you get at most 1–2 cells active at any moment regardless of how hard the object
presses. The `gamma` parameter controls sharpness, not count.

### The working approach: individual `<touch>` sensors

Inspired by the Shadow Hand in `HandManipulateEgg-v1` (Gymnasium Robotics), which uses
92 independent `<touch>` sensor sites. Each site is a **sphere** that fires independently
when any contact point falls within its volume. Placing many overlapping sites on a grid
achieves realistic spatial activation without fighting the contact model.

**Implementation** (`scripts/orange_individual_touch.py`):

- 16×16 sphere sites injected into link7 and link8 at runtime via XML manipulation
- Site pitch: 1.875 mm (X) × 2.8 mm (Y) across the 30×45 mm contact face
- Site radius: 4 mm — enough overlap to catch contacts reliably at each cell location
- Contact filtering: orange geoms use `contype=2 / conaffinity=2`; finger pads use
  `contype=3 / conaffinity=3` — orange only collides with the two finger pads, not the
  wrist or arm body
- Motion: orange_9 object (16 convex-decomp geoms, scale=0.040 ≈ 25 mm radius)
  oscillated between the pads with a slow SHM controller plus XZ sweep

**Results with orange_9 object:**

| Metric | Value |
|---|---|
| Peak cells active (left pad) | 18 / 256 |
| Peak force per cell | ~1.1 N |
| Simulation speed (no render) | **3.7× realtime** |
| Wall time for 15 s simulation | ~4 s |
| Wall time with video render | ~98 s (92% is ffmpeg encode) |

**Usage:**

```bash
# Headless benchmark (fast — ~4 s)
python scripts/orange_individual_touch.py

# Render annotated mp4 → scripts/orange_16x16_touch.mp4
python scripts/orange_individual_touch.py --render
```

The video shows the 3D gripper scene alongside two 16×16 heatmaps, one per finger,
with a live HUD showing active cell count and contact count per frame.

### Timing breakdown (no render)

```
sim (mj_step):   2.4 s  (60%)   ~82 µs/step    3.7× realtime
sensor read  :   0.15 s  (4%)   ~0.33 ms/read  (512 sensors)
other        :   1.5 s  (37%)   Python loop overhead
```

### Comparison table

| Approach | Multi-cell? | Speed | Notes |
|---|---|---|---|
| `touch_grid` (gamma=0) | 1–2 cells max | fast | Limited by point-contact sparsity |
| Individual `<touch>` sites | **15–18 cells** | 3.7× RT | This repo — recommended |
| MuJoCo `tactile` sensor (≥3.3.5) | native SDF-based | fast | Best long-term option; requires upgrade |

### Future: MuJoCo `tactile` sensor

MuJoCo 3.3.5 introduced a new `tactile` sensor type that uses SDF-based penetration
depth at user-defined grid points — bypassing contact sparsity entirely and giving dense
readings proportional to indentation depth. Upgrading to this sensor would be the
highest-fidelity path for real tactile data collection.

## Gain Tuning

Position controller gains (`kp`, `kv`) for each Piper joint are tuned via an interactive GUI before deploying to simulation or hardware.

### Tool: `includes/gain_tuner/tune_gains.py`

Loads the bare `piper_description.xml`, presents sliders for `kp` and `kv` per joint, runs a step-response simulation, and plots reference vs actual position.

**Why a separate tuner?**
The full robocasa kitchen environment (arm + Omron mobile base + scene) has complex coupled dynamics that make oscillation root-cause analysis difficult. The tuner uses only the bare arm on a fixed base to isolate joint behaviour, then the tuned gains are transferred back.

**Key simulation settings (matched to `send_joint_cmd.py`):**

| Setting | Value | Reason |
|---|---|---|
| Timestep | 0.001 s | Halves robocasa default; reduces discretisation error at high `kp` |
| Integrator | `IMPLICITFAST` | Adds implicit velocity damping — eliminates stiffness-driven oscillation |
| Solver iterations | 20 | Better constraint resolution per step |
| Start pose | `init_qpos` from `PiperOmron` | Matches real env starting configuration |

**Usage:**

```bash
cd includes/gain_tuner
python tune_gains.py
# point at a different XML
python tune_gains.py path/to/robot.xml
```

Adjust `kp` / `kv` sliders → set target position → click **Run Simulation** → inspect plots → click **Export XML** to save tuned gains back to file.

---

## Motion Profiling

Raw step commands (jump directly to target) cause large instantaneous position errors, saturating actuators and exciting structural oscillations. All motion in this repo uses a **trapezoidal velocity profile**.

### How it works

For each waypoint, a per-joint profile is computed:

```
Phase 1 — Accelerate  : q(t) = q0 + ½ a t²
Phase 2 — Cruise      : q(t) = q_acc + v_max (t - t_acc)
Phase 3 — Decelerate  : mirror of acceleration
```

All joints are **time-synchronised** — the slowest joint (longest travel) sets the total duration, and every other joint scales its cruise velocity down to match. This keeps the end-effector on a straight joint-space path.

A **triangular profile** is used automatically when the distance is too short to reach `max_vel` (avoids overshoot on small moves).

**Additionally**, a configurable settle period holds the robot at `init_qpos` before the first waypoint fires, allowing startup transients from the physics engine to decay.

### Parameters (set in `scripts/config/piper_behavior.json`)

| Parameter | Default | Description |
|---|---|---|
| `max_vel` | 0.4 rad/s | Peak joint velocity during cruise |
| `max_acc` | 0.3 rad/s² | Acceleration / deceleration ramp rate |
| `settle_time` | 1.0 s | Hold time at start before first move |
| `atol` | 0.01 rad | Convergence tolerance to declare waypoint reached |

---

## Behavior Config (`scripts/config/piper_behavior.json`)

All runtime parameters are centralised in a single JSON file. CLI flags act as per-run overrides without modifying the file.

```json
{
    "sim": {
        "timestep": 0.001,
        "integrator": "IMPLICITFAST",
        "iterations": 20
    },
    "motion": {
        "settle_time": 1.0,
        "duration": 10.0,
        "max_vel": 0.4,
        "max_acc": 0.3,
        "atol": 0.01
    },
    "robot": {
        "env_name": "PickPlaceCounterToCabinet",
        "init_qpos": [0.0, 1.57, -1.57, 0.0, 1.22, 0.0, 0.0, 0.0]
    },
    "waypoints": [
        [0.0,  0.5, -1.0,  0.0,  0.8,  0.0],
        [0.5,  1.0, -1.5,  0.3,  0.5, -0.5],
        [0.0,  1.57, -1.57, 0.0, 1.22,  0.0]
    ]
}
```

### Config sections

| Section | Purpose |
|---|---|
| `sim` | Physics engine settings — timestep, integrator, solver iterations |
| `motion` | Trapezoidal profile parameters and convergence tolerance |
| `robot` | Environment name and arm starting pose |
| `waypoints` | Ordered list of joint-space targets `[j1..j6]` in radians |

### Running

```bash
# Execute all waypoints from config
python scripts/send_joint_cmd.py

# Override a single parameter without editing the file
python scripts/send_joint_cmd.py --max-vel 0.2 --settle 2.0

# Single target (overrides config waypoints)
python scripts/send_joint_cmd.py --joints 0 0.5 -1.0 0 0.8 0

# Use a different config file
python scripts/send_joint_cmd.py --config scripts/config/pick_place.json
```

The config is designed to grow — future sections for IK targets, Cartesian waypoints, and task parameters can be added without changing the script interface.

---

## Sim-to-Real (Roadmap)
- Domain randomization via MuJoCo/robocasa
- Policy transfer from simulation to the physical Piper arm
- Calibration and state estimation utilities

## Project Structure

```
manipulation_vision/
├── scripts/
│   ├── config/
│   │   └── piper_behavior.json     # Centralised behavior config (sim, motion, waypoints)
│   ├── send_joint_cmd.py           # Joint-space waypoint execution with trapezoidal profiling
│   ├── orange_individual_touch.py  # 16×16 tactile grid demo
│   └── rgbd_stream.py              # RGB-D point cloud streaming
├── includes/
│   ├── gain_tuner/
│   │   └── tune_gains.py           # Interactive kp/kv tuning GUI
│   ├── robosuite/                  # Robot simulation framework with PiperArm/PiperOmron (fork)
│   ├── robocasa/                   # Kitchen environments and assets (fork)
│   └── mujoco-py/                  # Legacy mujoco-py bindings (reference)
├── src/
│   └── piper_ros/                  # ROS Noetic workspace for Piper arm (submodule)
│       ├── piper_description/      # URDF and MuJoCo model for Piper arm
│       ├── piper_sim/              # MuJoCo simulation nodes
│       └── piper_moveit/           # MoveIt configuration
└── README.md
```

## Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- `mujoco` (modern bindings)
- `python-can`, `piper_sdk`

## Installation

**1. Clone and initialize submodules**

```bash
git clone https://github.com/dt1729/manipulation_vision.git
cd manipulation_vision
git submodule update --init --recursive
```

**2. Set up robosuite and robocasa**

```bash
pip install -e includes/robosuite
pip install -e includes/robocasa
```

**3. Run a kitchen demo with PiperOmron**

```bash
python -m robocasa.demos.demo_kitchen_states --task PnPCounterToCab --robot PiperOmron
```

**4. Set up piper_ros (for real hardware)**

```bash
cd src/piper_ros
pip install python-can piper_sdk
catkin_make
source devel/setup.bash
```

## License

MIT License — Copyright 2026 Divya Tiwari
