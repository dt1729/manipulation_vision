---
type: module
module: test
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/robot_trajectory/test
file_count: 1
class_count: 7
boundary_count: 8
grey_count: 0
internal_count: 10
tags: [module, test]
---

# test

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/robot_trajectory/test`

| | Count |
|-|-------|
| 🔴 Boundary | 8 |
| 🟡 Grey | 0 |
| ⚪ Internal | 10 |
| 🟣 Classes | 7 |
| 📄 Files | 1 |

## External Dependencies

- [[libraries/gtest|gtest]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `modifyFirstWaypointAndCheckTrajectory` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `initTestTrajectory` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `copyTrajectory` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `modifyFirstWaypointPtrAndCheckTrajectory` | `test_robot_trajectory.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |

## Files

- `test_robot_trajectory.cpp`  `18 fn`  🔴 8 boundary
