import mujoco
import numpy as np
import pytest

from manipulation_vision.tactile_pnp import cameras
from manipulation_vision.tactile_pnp import scene as sc


@pytest.fixture(scope="module")
def built() -> sc.Scene:
    return sc.build_scene(sc.sample_task(np.random.default_rng(0)))


@pytest.fixture(scope="module")
def frames(built) -> dict[str, cameras.Frame]:
    with cameras.CameraRig(built.model, width=96, height=64) as rig:
        return rig.capture(built.data)


def test_capture_returns_both_cameras_with_requested_shapes(frames):
    assert list(frames) == ["scene_cam", "wrist_cam"]
    for f in frames.values():
        assert f.rgb.shape == (64, 96, 3) and f.rgb.dtype == np.uint8
        assert f.depth.shape == (64, 96) and f.depth.dtype == np.float32


def test_scene_camera_depth_is_metric(frames):
    # the optical axis hits the table at SCENE_CAM_TARGET (nothing taller stands there)
    d = frames["scene_cam"].depth
    expected = float(np.linalg.norm(sc.SCENE_CAM_POS - sc.SCENE_CAM_TARGET))
    assert d[32, 48] == pytest.approx(expected, abs=0.02)


def test_wrist_camera_image_changes_when_arm_moves(built):
    m, d, h = built.model, built.data, built.handles
    with cameras.CameraRig(m, width=64, height=64) as rig:
        before = rig.capture(d)["wrist_cam"].rgb.astype(int)
        d.qpos[h.arm_qpos[0]] += 0.5
        mujoco.mj_forward(m, d)
        after = rig.capture(d)["wrist_cam"].rgb.astype(int)
        d.qpos[h.arm_qpos[0]] -= 0.5
        mujoco.mj_forward(m, d)
    assert np.abs(after - before).mean() > 5


def test_width_beyond_offscreen_buffer_is_rejected(built):
    with pytest.raises(ValueError, match="offscreen"):
        cameras.CameraRig(built.model, width=10_000, height=64)


def test_close_is_idempotent_and_capture_after_close_raises(built):
    rig = cameras.CameraRig(built.model, width=32, height=32)
    rig.close()
    rig.close()
    with pytest.raises(RuntimeError, match="close"):
        rig.capture(built.data)
