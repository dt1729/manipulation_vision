---
type: module
module: test
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_planners/ompl/ompl_interface/test
file_count: 3
class_count: 21
boundary_count: 8
grey_count: 1
internal_count: 43
tags: [module, test]
---

# test

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_planners/ompl/ompl_interface/test`

| | Count |
|-|-------|
| 🔴 Boundary | 8 |
| 🟡 Grey | 1 |
| ⚪ Internal | 43 |
| 🟣 Classes | 21 |
| 📄 Files | 3 |

## External Dependencies

- [[libraries/gtest|gtest]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `testPathConstraints` | `test_planning_context_manager.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `testSimpleRequest` | `test_planning_context_manager.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_state_space.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_state_space.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `testJointLimits` | `test_state_validity_checker.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `testSelfCollision` | `test_state_validity_checker.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `testPathConstraints` | `test_state_validity_checker.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `setupPlanningContext` | `test_state_validity_checker.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |

## Files

- `test_planning_context_manager.cpp`  `18 fn`  🔴 2 boundary
- `test_state_space.cpp`  `6 fn`  🔴 2 boundary
- `test_state_validity_checker.cpp`  `28 fn`  🔴 4 boundary
