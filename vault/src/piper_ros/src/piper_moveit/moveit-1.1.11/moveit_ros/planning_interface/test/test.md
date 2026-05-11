---
type: module
module: test
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_ros/planning_interface/test
file_count: 10
class_count: 15
boundary_count: 12
grey_count: 0
internal_count: 61
tags: [module, test]
---

# test

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_ros/planning_interface/test`

| | Count |
|-|-------|
| 🔴 Boundary | 12 |
| 🟡 Grey | 0 |
| ⚪ Internal | 61 |
| 🟣 Classes | 15 |
| 📄 Files | 10 |

## External Dependencies

- [[libraries/gtest|gtest]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `synchronizeGeometryUpdate` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `planAndMoveToPose` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `planAndMove` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `testEigenPose` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `testPose` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest |
| `testJointPositions` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `move_group_interface_cpp_test.cpp` | gtest, gtest |
| `TestBody` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `move_group_interface_cpp_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `move_group_pick_place_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `subframes_test.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |

## Files

- `cleanup.py`  `4 fn`
- `dual_arm_robot_state_update.py`  `4 fn`
- `move_group_interface_cpp_test.cpp`  `15 fn`  🔴 10 boundary
- `move_group_pick_place_test.cpp`  `3 fn`  🔴 1 boundary
- `python_move_group.py`  `7 fn`
- `python_move_group_ns.py`  `6 fn`
- `robot_state_update.py`  `4 fn`
- `serialize_msg.py`  `13 fn`
- `serialize_msg_python_cpp_helpers.cpp`  `13 fn`
- `subframes_test.cpp`  `4 fn`  🔴 1 boundary
