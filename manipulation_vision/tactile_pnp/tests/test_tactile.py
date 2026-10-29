import dataclasses

import mujoco
import numpy as np
import pytest

from manipulation_vision.tactile_pnp import mjutil, tactile
from manipulation_vision.tactile_pnp import scene as sc


def _scene_with_cube_between_pads(half: float = 0.02) -> sc.Scene:
    task = sc.sample_task(np.random.default_rng(3))
    obj = dataclasses.replace(task.obj, kind="box", half_extents=(half, half, half))
    s = sc.build_scene(dataclasses.replace(task, obj=obj))
    m, d, h = s.model, s.data, s.handles
    m.opt.gravity[:] = 0.0
    hand = mjutil.body_id(m, "right_hand")
    pads = [mjutil.site_id(m, n) for n in ("tac_pad_left", "tac_pad_right")]
    d.qpos[h.obj_qpos:h.obj_qpos + 3] = d.site_xpos[pads].mean(axis=0)
    d.qpos[h.obj_qpos + 3:h.obj_qpos + 7] = d.xquat[hand]
    mujoco.mj_forward(m, d)
    return s


def _squeeze(s: sc.Scene, seconds: float = 0.3) -> None:
    # command 5 mm past the cube faces (~50 N per finger at kp 1e4); the cube stops the pads
    s.data.ctrl[s.handles.finger_act] = sc.FINGER_SIGN * 0.015
    for _ in range(int(seconds / s.model.opt.timestep)):
        mujoco.mj_step(s.model, s.data)


@pytest.fixture(scope="module")
def squeezed() -> sc.Scene:
    s = _scene_with_cube_between_pads()
    _squeeze(s)
    return s


def test_grids_read_zero_with_open_gripper_in_free_space():
    s = sc.build_scene(sc.sample_task(np.random.default_rng(0)))
    mujoco.mj_step(s.model, s.data)
    assert not tactile.read_grids(s.data, s.tactile_adr).any()


def test_grid_shape_is_two_pads_of_16x16(squeezed):
    assert tactile.read_grids(squeezed.data, squeezed.tactile_adr).shape == (2, 16, 16)


def test_squeezing_a_cube_activates_both_pads(squeezed):
    cells = tactile.active_cells(tactile.read_grids(squeezed.data, squeezed.tactile_adr))
    assert (cells > 0).all(), cells


def test_finger_contact_forces_positive_on_both_fingers_when_squeezing(squeezed):
    h = squeezed.handles
    f = tactile.finger_contact_forces(squeezed.model, squeezed.data, h.finger_bodies, h.obj_geom)
    assert (f > 1.0).all(), f


def test_finger_contact_forces_zero_without_contact():
    s = sc.build_scene(sc.sample_task(np.random.default_rng(0)))
    h = s.handles
    f = tactile.finger_contact_forces(s.model, s.data, h.finger_bodies, h.obj_geom)
    assert f.tolist() == [0.0, 0.0]


def test_pad_contact_false_for_zero_grid():
    assert tactile.pad_contact(np.zeros((2, 16, 16))).tolist() == [False, False]


def test_pad_contact_true_for_one_cell_above_threshold():
    g = np.zeros((2, 16, 16))
    g[1, 4, 5] = 1.0
    assert tactile.pad_contact(g).tolist() == [False, True]


def test_pad_contact_respects_min_cells():
    g = np.zeros((2, 16, 16))
    g[0, 0, :3] = 1.0
    assert tactile.pad_contact(g, min_cells=4).tolist() == [False, False]


def test_grid_without_pad_axis_is_rejected():
    with pytest.raises(ValueError, match="shape"):
        tactile.active_cells(np.zeros((16, 16)))


def test_negative_threshold_is_rejected():
    with pytest.raises(ValueError, match="threshold"):
        tactile.active_cells(np.zeros((2, 16, 16)), threshold=-1.0)
