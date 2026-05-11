---
type: module
module: test
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/trajectory_processing/test
file_count: 4
class_count: 14
boundary_count: 13
grey_count: 0
internal_count: 17
tags: [module, test]
---

# test

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/trajectory_processing/test`

| | Count |
|-|-------|
| 🔴 Boundary | 13 |
| 🟡 Grey | 0 |
| ⚪ Internal | 17 |
| 🟣 Classes | 14 |
| 📄 Files | 4 |

## External Dependencies

- [[libraries/gtest|gtest]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `TestBody` | `test_limit_cartesian_speed.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_ruckig_traj_smoothing.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_ruckig_traj_smoothing.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_ruckig_traj_smoothing.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_optimal_trajectory_generation.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_optimal_trajectory_generation.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_optimal_trajectory_generation.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_optimal_trajectory_generation.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_optimal_trajectory_generation.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_parameterization.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_parameterization.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_parameterization.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_time_parameterization.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |

## Files

- `test_limit_cartesian_speed.cpp`  `4 fn`  🔴 1 boundary
- `test_ruckig_traj_smoothing.cpp`  `7 fn`  🔴 3 boundary
- `test_time_optimal_trajectory_generation.cpp`  `10 fn`  🔴 5 boundary
- `test_time_parameterization.cpp`  `9 fn`  🔴 4 boundary
