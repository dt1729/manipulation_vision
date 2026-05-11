---
type: module
module: src
path: src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/collision_detection/src
file_count: 6
class_count: 1
boundary_count: 3
grey_count: 0
internal_count: 10
tags: [module, src]
---

# src

> Drill deeper: `csviz vault . --expand src/piper_ros/src/piper_moveit/moveit-1.1.11/moveit_core/collision_detection/src`

| | Count |
|-|-------|
| 🔴 Boundary | 3 |
| 🟡 Grey | 0 |
| ⚪ Internal | 10 |
| 🟣 Classes | 1 |
| 📄 Files | 6 |

## External Dependencies

- [[libraries/octomap|octomap]]

## Boundary Surface

| Function | File | External calls |
|----------|------|----------------|
| `getMetaballSurfaceProperties` | `collision_octomap_filter.cpp` | octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap |
| `findSurface` | `collision_octomap_filter.cpp` | octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap, octomap |
| `sampleCloud` | `collision_octomap_filter.cpp` | octomap, octomap, octomap, octomap, octomap, octomap |

## Files

- `collision_env.cpp`  `2 fn`
- `collision_matrix.cpp`  `1 fn`
- `collision_octomap_filter.cpp`  `3 fn`  🔴 3 boundary
- `collision_plugin_cache.cpp`  `0 fn`
- `collision_tools.cpp`  `6 fn`
- `pycollision_detection.cpp`  `1 fn`
