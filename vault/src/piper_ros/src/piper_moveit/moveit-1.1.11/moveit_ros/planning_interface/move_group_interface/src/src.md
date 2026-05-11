---
type: module
module: src
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_ros/planning_interface/move_group_interface/src
file_count: 3
class_count: 5
boundary_count: 5
grey_count: 0
internal_count: 68
tags: [module, src]
---

# src

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_ros/planning_interface/move_group_interface/src`

| | Count |
|-|-------|
| 🔴 Boundary | 5 |
| 🟡 Grey | 0 |
| ⚪ Internal | 68 |
| 🟣 Classes | 5 |
| 📄 Files | 3 |

## External Dependencies

- [[libraries/boost|boost]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `getJointValueTargetPythonList` | `wrap_python_move_group.cpp` | boost, boost, boost, boost |
| `getRememberedJointValuesPython` | `wrap_python_move_group.cpp` | boost, boost, boost, boost |
| `getCurrentStateBoundedPython` | `wrap_python_move_group.cpp` | boost, boost, boost, boost |
| `getNamedTargetValuesPython` | `wrap_python_move_group.cpp` | boost, boost, boost, boost |
| `getJacobianMatrixPython` | `wrap_python_move_group.cpp` | boost, boost |

## Files

- `demo.cpp`  `3 fn`
- `move_group_interface.cpp`  `1 fn`
- `wrap_python_move_group.cpp`  `69 fn`  🔴 5 boundary
