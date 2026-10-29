import dataclasses
from pathlib import Path

import mujoco
import numpy as np
import pytest

from manipulation_vision.tactile_pnp import scene as sc


@pytest.fixture(scope="module")
def task() -> sc.TaskSpec:
    return sc.sample_task(np.random.default_rng(0))


@pytest.fixture(scope="module")
def built(task) -> sc.Scene:
    return sc.build_scene(task)


def test_scene_has_512_touch_sensors(built):
    m = built.model
    n_touch = sum(int(m.sensor_type[i] == mujoco.mjtSensor.mjSENS_TOUCH) for i in range(m.nsensor))
    assert n_touch == 512


def test_tactile_addresses_cover_two_distinct_16x16_grids(built):
    adr = built.tactile_adr
    assert adr.shape == (2, 16, 16)
    assert len(np.unique(adr)) == 512


def test_touch_grid_plugin_sensors_are_removed(built):
    assert built.model.nplugin == 0


def test_object_rests_on_table(built):
    m, d = built.model, built.data
    start = d.xpos[built.handles.obj_body].copy()
    for _ in range(200):
        mujoco.mj_step(m, d)
    moved = np.linalg.norm(d.xpos[built.handles.obj_body] - start)
    assert moved < 0.002


def test_equal_seeds_give_equal_tasks():
    a = sc.sample_task(np.random.default_rng(42))
    b = sc.sample_task(np.random.default_rng(42))
    assert a == b


def test_different_seeds_give_different_tasks():
    a = sc.sample_task(np.random.default_rng(1))
    b = sc.sample_task(np.random.default_rng(2))
    assert a != b


def test_place_target_is_at_least_min_distance_from_object():
    ws = sc.Workspace()
    for seed in range(50):
        t = sc.sample_task(np.random.default_rng(seed), ws)
        d = np.hypot(t.place_xy[0] - t.obj.pos[0], t.place_xy[1] - t.obj.pos[1])
        assert d >= ws.min_place_dist


def test_zero_mass_is_rejected(task):
    bad = dataclasses.replace(task, obj=dataclasses.replace(task.obj, mass=0.0))
    with pytest.raises(ValueError, match="mass"):
        sc.build_scene(bad)


def test_unsatisfiable_place_distance_is_rejected():
    ws = sc.Workspace(x=(0.3, 0.31), y=(0.0, 0.01), min_place_dist=0.2)
    with pytest.raises(ValueError, match="min_place_dist"):
        sc.sample_task(np.random.default_rng(0), ws)


def test_build_writes_nothing_into_robot_asset_dir(task):
    asset_dir = Path(sc.ROBOT_XML).parent
    before = set(asset_dir.iterdir())
    sc.build_scene(task)
    assert set(asset_dir.iterdir()) == before


def test_missing_robot_file_raises_file_not_found(task, monkeypatch, tmp_path):
    monkeypatch.setattr(sc, "ROBOT_XML", tmp_path / "nope.xml")
    with pytest.raises(FileNotFoundError):
        sc.build_xml(task)


def test_build_xml_declares_one_touch_sensor_per_taxel(task):
    xml = sc.build_xml(task)
    assert xml.count("<touch ") == 512
    assert "touch_grid" not in xml


def test_pad_layout_sites_stay_inside_the_pad():
    pad = sc.PadLayout()
    xs, ys = pad.site_positions()
    assert len(xs) == pad.nx and len(ys) == pad.ny
    assert np.abs(xs).max() < pad.half_x
    assert np.abs(ys - pad.center_y).max() < pad.half_y
    assert np.diff(xs).min() > pad.site_radius  # a site never reaches its neighbour's bump


def test_sphere_half_height_is_its_radius():
    o = sc.ObjectSpec("sphere", (0.03, 0.03, 0.03), 0.1, 1.0, (0.3, 0.0, 0.03), 0.0)
    assert o.half_height == 0.03


def test_workspace_with_inverted_bounds_is_rejected():
    with pytest.raises(ValueError, match=r"Workspace\.mass"):
        sc.Workspace(mass=(0.5, 0.1)).validate()


def test_handles_resolve_six_distinct_arm_joints(built):
    h: sc.Handles = built.handles
    assert len(set(h.arm_qpos.tolist())) == 6
    assert h.finger_bodies[0] != h.finger_bodies[1]


def test_reset_to_home_restores_posture_and_open_gripper(built):
    h = built.handles
    built.data.qpos[h.arm_qpos] += 0.3
    sc.reset_to_home(built)
    np.testing.assert_allclose(built.data.qpos[h.arm_qpos], sc.HOME_Q)
    np.testing.assert_allclose(np.abs(built.data.qpos[h.finger_qpos]), sc.GRIPPER_MAX_OPENING)


def test_lookat_axes_point_camera_at_target():
    pos, target = np.array([1.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.0])
    x, y = np.array(sc.lookat_xyaxes(pos, target).split(), float).reshape(2, 3)
    view = -np.cross(x, y)  # cameras look along -z
    np.testing.assert_allclose(view, (target - pos) / np.linalg.norm(target - pos), atol=1e-5)


@pytest.mark.parametrize("kind", ["box", "cylinder", "sphere"])
def test_every_object_kind_compiles(task, kind):
    obj = dataclasses.replace(task.obj, kind=kind, half_extents=(0.02, 0.02, 0.03))
    s = sc.build_scene(dataclasses.replace(task, obj=obj))
    assert s.model.body_mass[s.handles.obj_body] == pytest.approx(obj.mass)
