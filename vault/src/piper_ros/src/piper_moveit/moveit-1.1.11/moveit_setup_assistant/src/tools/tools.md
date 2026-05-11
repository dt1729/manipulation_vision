---
type: module
module: tools
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_setup_assistant/src/tools
file_count: 7
class_count: 2
boundary_count: 3
grey_count: 0
internal_count: 55
tags: [module, tools]
---

# tools

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_setup_assistant/src/tools`

| | Count |
|-|-------|
| 🔴 Boundary | 3 |
| 🟡 Grey | 0 |
| ⚪ Internal | 55 |
| 🟣 Classes | 2 |
| 📄 Files | 7 |

## External Dependencies

- [[libraries/boost|boost]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `computeDefaultCollisions` | `compute_default_collisions.cpp` | boost, boost, boost, boost, boost, boost, boost, boost |
| `disableNeverInCollision` | `compute_default_collisions.cpp` | boost, boost, boost, boost, boost, boost, boost, boost, boost, boost, boost, boost |
| `disableNeverInCollisionThread` | `compute_default_collisions.cpp` | boost, boost |

## Files

- `collision_linear_model.cpp`  `21 fn`
- `collision_matrix_model.cpp`  `13 fn`
- `compute_default_collisions.cpp`  `13 fn`  🔴 3 boundary
- `moveit_config_data.cpp`  `1 fn`
- `rotated_header_view.cpp`  `4 fn`
- `xml_manipulation.cpp`  `2 fn`
- `xml_syntax_highlighter.cpp`  `4 fn`
