---
type: module
module: test
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_kinematics/test
file_count: 1
class_count: 10
boundary_count: 14
grey_count: 3
internal_count: 13
tags: [module, test]
---

# test

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_kinematics/test`

| | Count |
|-|-------|
| 🔴 Boundary | 14 |
| 🟡 Grey | 3 |
| ⚪ Internal | 13 |
| 🟣 Classes | 10 |
| 📄 Files | 1 |

## External Dependencies

- [[libraries/gtest|gtest]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `initialize` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `SetUp` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `isNear` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `expectNearHelper` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `searchIKCallback` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `parseVector` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_kinematics_plugin.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |

## Files

- `test_kinematics_plugin.cpp`  `30 fn`  🔴 14 boundary
