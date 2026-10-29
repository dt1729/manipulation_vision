import imageio.v2 as imageio
import mujoco
import numpy as np
import pytest

from manipulation_vision.tactile_pnp import pnp, preview
from manipulation_vision.tactile_pnp import scene as sc


def test_zero_grid_heatmap_is_black_with_scaled_shape():
    img = preview.heatmap(np.zeros((16, 16)), vmax=1.0, cell_px=4)
    assert img.shape == (64, 64, 3)
    assert img.dtype == np.uint8
    assert img.max() < 10  # inferno(0) is near-black


def test_heatmap_hottest_where_grid_peaks():
    g = np.zeros((16, 16))
    g[3, 12] = 1.0
    img = preview.heatmap(g, vmax=1.0, cell_px=4).astype(int).sum(axis=2)
    y, x = np.unravel_index(np.argmax(img), img.shape)
    assert (y // 4, x // 4) == (3, 12)


def test_three_d_grid_is_rejected():
    with pytest.raises(ValueError, match="shape"):
        preview.heatmap(np.zeros((2, 16, 16)), vmax=1.0)


def test_preview_writer_close_finalises_a_playable_mp4(tmp_path):
    s = sc.build_scene(sc.sample_task(np.random.default_rng(0)))
    path = tmp_path / "p.mp4"
    with preview.PreviewWriter(s.model, path, fps=50, control_hz=100.0, size=(160, 120)) as pw:
        for k in range(10):
            pw(s, pnp.StepInfo(t=k * 0.01, phase=pnp.Phase.GRASP, action=np.zeros(7),
                               tactile=np.zeros((2, 16, 16), np.float32)))
    assert pw.frames == 5
    pw.close()  # already closed by the context manager: must be a no-op
    frames = imageio.mimread(path)
    assert len(frames) == 5


def test_fps_above_control_rate_is_rejected(tmp_path):
    m = mujoco.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>")
    with pytest.raises(ValueError, match="fps"):
        preview.PreviewWriter(m, tmp_path / "x.mp4", fps=200, control_hz=100.0)
