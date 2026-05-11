---
type: module
module: test
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/planning_scene/test
file_count: 3
class_count: 15
boundary_count: 17
grey_count: 0
internal_count: 23
tags: [module, test]
---

# test

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/planning_scene/test`

| | Count |
|-|-------|
| 🔴 Boundary | 17 |
| 🟡 Grey | 0 |
| ⚪ Internal | 23 |
| 🟣 Classes | 15 |
| 📄 Files | 3 |

## External Dependencies

- [[libraries/gtest|gtest]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `TestBody` | `test_collision_objects.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_collision_objects.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_collision_objects.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `gtest_PluginTestsCollisionDetectorTests_EvalGenerateName_` | `test_multi_threaded.cpp` | gtest, gtest, gtest, gtest |
| `runCollisionDetectionAssert` | `test_multi_threaded.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `SetUp` | `test_multi_threaded.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_multi_threaded.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |
| `TestBody` | `test_planning_scene.cpp` | gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest, gtest |

## Files

- `test_collision_objects.cpp`  `7 fn`  🔴 3 boundary
- `test_multi_threaded.cpp`  `9 fn`  🔴 4 boundary
- `test_planning_scene.cpp`  `24 fn`  🔴 10 boundary
