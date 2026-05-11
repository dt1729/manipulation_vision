---
type: module
module: src
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_ros/planning_interface/robot_interface/src
file_count: 1
class_count: 1
boundary_count: 10
grey_count: 0
internal_count: 22
tags: [module, src]
---

# src

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_ros/planning_interface/robot_interface/src`

| | Count |
|-|-------|
| 🔴 Boundary | 10 |
| 🟡 Grey | 0 |
| ⚪ Internal | 22 |
| 🟣 Classes | 1 |
| 📄 Files | 1 |

## External Dependencies

- [[libraries/boost|boost]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `getGroupJointTips` | `wrap_python_robot_interface.cpp` | boost, boost |
| `getGroupLinkNames` | `wrap_python_robot_interface.cpp` | boost, boost |
| `getJointLimits` | `wrap_python_robot_interface.cpp` | boost, boost, boost, boost |
| `getLinkPose` | `wrap_python_robot_interface.cpp` | boost, boost, boost, boost, boost, boost |
| `getDefaultStateNames` | `wrap_python_robot_interface.cpp` | boost, boost, boost, boost |
| `getCurrentJointValues` | `wrap_python_robot_interface.cpp` | boost, boost, boost, boost, boost, boost |
| `getJointValues` | `wrap_python_robot_interface.cpp` | boost, boost |
| `getCurrentVariableValues` | `wrap_python_robot_interface.cpp` | boost, boost, boost, boost, boost, boost |
| `getGroupActiveJointNames` | `wrap_python_robot_interface.cpp` | boost, boost |
| `getGroupJointNames` | `wrap_python_robot_interface.cpp` | boost, boost |

## Files

- `wrap_python_robot_interface.cpp`  `32 fn`  🔴 10 boundary
