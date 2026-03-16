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

### Reinforcement Learning
- MuJoCo-based simulation environment (modern `mujoco` bindings)
- PID-controlled joint actuation as a baseline
- Gym-compatible interface for training RL policies

### VLA (Vision-Language-Action)
- Designed to support VLA model inference for manipulation tasks
- Vision input pipeline for perception-driven control
- Language-conditioned policy execution

## Sim-to-Real (Roadmap)
- Domain randomization support via MuJoCo
- Policy transfer from simulation to the physical Piper arm
- Calibration and state estimation utilities

## Project Structure

```
manipulation_vision/
├── src/
│   └── piper_ros/          # ROS Noetic workspace for Piper arm (submodule)
├── includes/
│   └── robocasa/           # Simulation assets and environments (submodule)
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

**2. Set up piper_ros**

```bash
cd src/piper_ros
pip install python-can piper_sdk
catkin_make
source devel/setup.bash
```

**3. Install mujoco**

```bash
pip install mujoco
```

## License

MIT License — Copyright 2026 Divya Tiwari
