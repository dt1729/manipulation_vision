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

## Sim-to-Real (Roadmap)
- Domain randomization via MuJoCo/robocasa
- Policy transfer from simulation to the physical Piper arm
- Calibration and state estimation utilities

## Project Structure

```
manipulation_vision/
├── src/
│   └── piper_ros/          # ROS Noetic workspace for Piper arm (submodule)
├── includes/
│   ├── robosuite/          # Robot simulation framework with PiperArm/PiperOmron (fork)
│   └── robocasa/           # Kitchen environments and assets (fork)
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
