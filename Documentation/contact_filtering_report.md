# Contact Filtering in robosuite / robocasa

**Date:** 2026-04-06  
**Context:** PiperOmron robot in robocasa kitchen environments; investigating why MuJoCo touch sensors read 0 despite finger geoms having `contype=1 conaffinity=1`.

---

## TL;DR

There is **no single global contact filter** that blanket-suppresses robot contacts. The real mechanism is a layered system of **per-geom `contype`/`conaffinity` bitmasks** applied during XML authoring. Robot arm links 0–6 are deliberately made non-colliding (`contype=0 conaffinity=0`). Only the fingertip geoms have collision enabled. Touch sensors read 0 because the teleported object lands at the palm mount point, not between the fingertips.

---

## 1. The Dual-Geom Pattern (Primary Mechanism)

Every model in robosuite/robocasa — robots, fixtures, objects, arenas — follows a strict two-geom-per-link convention:

| Geom role | `group` | `contype` | `conaffinity` | Purpose |
|-----------|---------|-----------|---------------|---------|
| **Visual** | `1` | `0` | `0` | High-fidelity mesh for rendering only |
| **Collision** | `0` (default) | `1` (default) | `1` (default) | Simplified shape for physics |

**MuJoCo contact rule:** a contact between geom A and geom B is generated only if  
`(A.contype & B.conaffinity) != 0 AND (B.contype & A.conaffinity) != 0`

Setting both to `0` completely eliminates that geom from the physics engine's contact pipeline while keeping it visible in the renderer.

**Why:** Full mesh geometry collision is numerically unstable and 10–100× more expensive than simplified primitives. Using box/capsule/cylinder collision proxies gives stable dynamics at a fraction of the cost.

**Files that apply this pattern:**
- All robot arm link meshes: `robosuite/models/assets/robots/piper_arm/robot.xml` lines 23–55
- Arena walls: `robosuite/models/assets/arenas/empty_arena.xml` lines 15–20 (and every other arena XML)
- Kitchen fixture visual meshes: `robocasa/models/assets/fixtures/**/*.xml` — all visual mesh geoms set `contype="0" conaffinity="0" group="1"`
- Object visual geoms: `robosuite/models/objects/objects.py:465–468` (Python sets these at construction)

---

## 2. Robot Arm Link Contact Disabling

Specific to robots, the arm links (links 0–6 on PiperArm) have **both** their visual and collision-proxy geoms set to `contype="0" conaffinity="0"`:

```xml
<!-- robot.xml — link2 representative example -->
<geom type="mesh" mesh="link2" contype="0" conaffinity="0" group="1"/>
<!-- No separate collision geom for link2 — arm doesn't collide with scene -->
```

Only the gripper fingertips get collision geoms:
```xml
<!-- link7 finger — visual (no collision) -->
<geom type="mesh" mesh="link7" contype="0" conaffinity="0" group="1"/>
<!-- link7 finger — collision enabled -->
<geom type="mesh" mesh="link7" name="link7_col" contype="1" conaffinity="1"/>
```

**Why the arm is disabled:**
1. **Performance** — the kitchen scene has many fixture bodies. Enabling arm-scene collision would add ~6 bodies × N fixture bodies worth of broadphase pairs, roughly doubling contact computation in a typical kitchen.
2. **Stability** — thin robot links at high servo gains + mesh-mesh contacts cause jitter and constraint explosions. Disabling them lets the controller focus on EEF contact forces only.
3. **Task scope** — kitchen manipulation tasks assume the arm doesn't collide with the scene during normal execution. The constraint is enforced by the task designer, not the simulator.

**Files:**
- `robosuite/models/assets/robots/piper_arm/robot.xml`: links 0–6 all `contype=0 conaffinity=0`
- Same pattern in `panda/robot.xml`, `gr1/robot.xml`, etc.

---

## 3. Kitchen Fixture Contact Handling

Kitchen fixtures follow a mixed strategy:

### 3a. Decorative fixtures (coffee machines, stovetops, ovens, etc.)
All geoms are `contype=0 conaffinity=0`:
```xml
<!-- CoffeeMachine082/model.xml -->
<geom conaffinity="0" contype="0" group="1" type="mesh"/>       <!-- visual mesh -->
<geom group="0" rgba="1 1 1 0.5" contype="0" conaffinity="0"/> <!-- "collision" — also disabled! -->
```
These fixtures are visual-only. Objects cannot physically rest on them; robocasa instead uses bounding-box or distance-based placement logic at reset time to position objects on top.

### 3b. Counter surfaces
Counters use chunked collision geoms with default contype=1:
```python
# counter.py:371–382
g = new_geom(
    name=geom_name + "_{}".format(i),
    type="box",
    size=chunk_sizes[i] / 2,
    group=0,        # collision group
    density=10,
    # NO contype/conaffinity specified → defaults to 1/1
)
self._contact_geoms.append("top_{}".format(i))
```
Counter tops are split into 0.5 m chunks, each a box collision geom. Objects rest on these physically. This is where the 300+ contacts in a kitchen scene come from — every object resting on every counter chunk generates an active contact.

### 3c. Why so many contacts (300+)?
In a full kitchen episode:
- N objects × M counter chunks = O(N×M) object-surface contacts
- Fixtures themselves may contact each other and the floor
- MuJoCo counts ALL contacts including static resting ones in `ncon`
- The robot arm contributes **zero** contacts (all links 0–6 are contype=0)

---

## 4. Python-Level Contact Checking — and Why It's Partially Broken

### Class hierarchy for `contact_geoms`

`MujocoModel` (abstract base, `base.py:396`) declares `contact_geoms` as an abstract property that raises `NotImplementedError`. It is an interface, not an implementation.

The concrete subclasses implement it:
- `MujocoXMLModel.contact_geoms` (`base.py:612`) — returns `self.correct_naming(self._contact_geoms)`
- `MujocoObject.contact_geoms` (`objects.py:224`) — same

Both classes populate `_contact_geoms` using `_element_filter` in `mjcf_utils.py`:

```python
# mjcf_utils.py:689-693
elif element.tag == "geom":
    group = element.get("group")
    if group in {None, "0", "1"}:
        return "visual_geoms" if group == "1" else "contact_geoms"
```

**Critical flaw:** classification is by `group` attribute only — it ignores `contype` and `conaffinity` entirely.

This means a geom like:
```xml
<geom group="0" rgba="1 1 1 0.5" contype="0" conaffinity="0"/>
```
is added to `_contact_geoms` in Python even though MuJoCo will never generate physics contacts for it.

### The `check_contact` function

```python
# sim_utils.py:8-40
def check_contact(sim, geoms_1, geoms_2=None):
    if isinstance(geoms_1, MujocoModel):
        geoms_1 = geoms_1.contact_geoms   # calls concrete implementation — no NotImplementedError
    ...
    for i in range(sim.data.ncon):
        contact = sim.data.contact[i]
        g1 = sim.model.geom_id2name(contact.geom1)
        g2 = sim.model.geom_id2name(contact.geom2)
        if (g1 in geoms_1 and g2 in geoms_2) or (g2 in geoms_1 and g1 in geoms_2):
            return True
    return False
```

`check_obj_fixture_contact` in `robocasa/utils/object_utils.py:623` delegates to this.

### Why it silently returns False for most fixtures

The function searches `sim.data.contact[:ncon]` for geom name matches. If MuJoCo never generated the contact (because the geom has `contype=0`), it won't be in that array, and the function returns `False` — no exception, no warning, just wrong results.

**What actually works vs. what silently fails:**

| Model | `contact_geoms` populated? | MuJoCo contacts generated? | `check_contact` works? |
|-------|---------------------------|---------------------------|----------------------|
| Counter chunks (`top_0`, `top_1`...) | Yes (group=0, no contype set → defaults 1/1) | Yes | **Yes** |
| Kitchen objects (mugs, cans, etc.) | Yes | Yes | **Yes** |
| Coffee machines, stovetops, ovens | Yes (group=0, but contype=0) | **No** | **No — silently returns False** |
| Robot arm links 0–6 (contype=0) | **Not in contact_geoms** — they have no separate collision geom | No | N/A |
| Robot fingertips (`link7_col`, `link8_col`) | Yes (contype=1) | Yes, if touching something | **Yes** |

The robot arm links 0–6 are different from fixtures: they have **no group=0 geom at all** (only a single mesh geom with `contype=0 conaffinity=0 group=1`). So they are not in `contact_geoms` and MuJoCo generates nothing.

**Bottom line:** `contact_geoms` in Python = "all group=0 geoms". It is NOT a reliable indicator of whether MuJoCo will generate contacts. The collision checking in robosuite only works correctly when the underlying MuJoCo geoms actually have `contype=1 conaffinity=1`.

---

## 5. MuJoCo Touch Sensors and Why They Read 0

### How touch sensors work
A `<touch>` sensor is attached to a `<site>`. It measures the **sum of contact normal forces on all geoms belonging to the site's parent body**. It does NOT care about the site's position or size.

```xml
<!-- sensor reads ALL contacts on body "link7" -->
<touch name="tac_left_00" site="tac_left_00"/>
```

For the sensor to fire:
1. The parent body's geom (`link7_col`) must have `contype=1 conaffinity=1` ✅ (we set this)
2. The contacting object's geom must also have `contype=1 conaffinity=1` ✅ (default for objects)
3. The geoms must **physically overlap** in the simulation ← this is the actual failure point

### Why the sensors read 0 in our test

When we teleport the object to the end-effector position:
```python
eef_pos = env.sim.data.get_body_xpos("robot0_right_hand").copy()
env.sim.data.qpos[qpos_addr[0]:qpos_addr[0]+3] = eef_pos
```

`robot0_right_hand` is the **palm mount body** — the anchor point for the finger bodies. The actual finger geoms (link7_col, link8_col) extend ~65 mm **forward** from this point along the finger axis. Teleporting to `eef_pos` places the object at the palm, not between the fingertips.

Result: the fingertip geoms never overlap the object, so MuJoCo generates no finger-object contacts, and the touch sensors read 0.

### The arm link filter compounds this
Even if an object somehow brushed the arm (links 0–6), those geoms have `contype=0` — MuJoCo generates zero contacts for them. The touch sensor architecture is correct; the issue is contact geometry placement.

---

## 6. Contact Exclusions

There are two explicit contact exclusions in the model:

```xml
<!-- piper_arm/robot.xml:114–116 -->
<contact>
    <exclude body1="link7" body2="link8"/>
</contact>
```

This prevents the two finger bodies from colliding with each other (they face inward and would intersect when closing).

robosuite's model assembler merges contact exclusions from all sub-models into the final MuJoCo XML via `MujocoModel.merge()` / `MujocoXML.merge()`.

No kitchen-specific contact exclusions were found for robot-object pairs.

---

## 7. Performance Optimization Summary

| Mechanism | Geoms affected | Physics cost saved |
|-----------|---------------|-------------------|
| Visual geoms (`contype=0`) | All visual meshes in every model | Broadphase elimination of all non-physics geometry |
| Arm link disabling (`contype=0`) | Robot links 0–6 (~6 bodies) | Eliminates O(6 × N_fixtures) contact pairs |
| Fixture decoration disabling | Coffee machines, stovetops, ovens | Eliminates complex mesh-mesh contacts for static objects |
| Counter chunking | Counter tops split into 0.5 m boxes | Replaces large single mesh with stable box contacts |
| Contact exclusion (link7↔link8) | Finger self-collision | Prevents invalid self-contacts during gripper close |

**Net result in a kitchen scene:** Physics contacts come almost entirely from (objects × counter surface chunks). The robot contributes only fingertip contacts, and decorative fixtures contribute nothing. This keeps `ncon` manageable at ~300 rather than thousands.

---

## 8. How to Enable Finger-Object Touch Sensors

The arm link and fixture disabling are not the blocker. The fix is ensuring the object is placed **between the fingertips**, not at the palm mount.

```python
# Find where the fingertips actually are
link7_pos = env.sim.data.get_body_xpos("robot0_link7")
link8_pos = env.sim.data.get_body_xpos("robot0_link8")
fingertip_center = (link7_pos + link8_pos) / 2

# Teleport object to fingertip center
env.sim.data.qpos[qpos_addr[0]:qpos_addr[0]+3] = fingertip_center
```

Alternatively, rather than teleporting, use the OSC controller to move the EEF to an object on the counter and close the gripper — the contact will form naturally and the touch sensors will fire.

The 6D force/torque sensor (`gripper0_right_force_ee`, `gripper0_right_torque_ee`) is the practical alternative for the VLA pipeline today, since it works regardless of object placement and confirmed returning ~310 N during our gripper-close test.

---

## 9. Files Reference

| File | Relevance |
|------|-----------|
| `robosuite/models/assets/robots/piper_arm/robot.xml` | Arm link contype=0, fingertip contype=1, touch sensor XML |
| `robosuite/models/assets/arenas/empty_arena.xml` | Arena wall visual geoms |
| `robocasa/models/fixtures/counter.py:350–382` | Counter collision chunk construction |
| `robocasa/models/assets/fixtures/*/model.xml` | Fixture geoms (all contype=0) |
| `robosuite/models/objects/objects.py:455–468` | Visual geom template (contype=0) |
| `robosuite/utils/sim_utils.py:8–67` | Python contact checking utilities |
| `robocasa/utils/object_utils.py:623–629` | `check_obj_fixture_contact` |
| `robocasa/environments/kitchen/kitchen.py:1216–1341` | `edit_model_xml` — camera injection, no contact manipulation |
