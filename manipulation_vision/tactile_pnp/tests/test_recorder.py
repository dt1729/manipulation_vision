import h5py
import mujoco
import numpy as np
import pytest

from manipulation_vision.tactile_pnp import cameras, pnp, recorder
from manipulation_vision.tactile_pnp import scene as sc

DT = 0.01


@pytest.fixture(scope="module")
def built() -> sc.Scene:
    return sc.build_scene(sc.sample_task(np.random.default_rng(0)))


def _feed(rec: recorder.EpisodeRecorder, s: sc.Scene, n: int) -> list[pnp.StepInfo]:
    infos = []
    for k in range(n):
        for _ in range(10):
            mujoco.mj_step(s.model, s.data)
        grid = np.zeros((2, 16, 16), np.float32)
        grid[0, k % 16, 0] = 1.0  # left pad marker
        grid[1, 0, k % 16] = 2.0  # right pad marker
        info = pnp.StepInfo(t=float(s.data.time), phase=pnp.Phase(min(k // 10, 9)),
                            action=np.full(7, float(k)), tactile=grid)
        rec(s, info)
        infos.append(info)
    return infos


def _result() -> pnp.EpisodeResult:
    return pnp.EpisodeResult(success=True, failure=None, place_error=0.01, sim_time=1.0,
                             peak_active={"GRASP": [3, 4]}, grasp_force=[1.0, 2.0])


@pytest.fixture(scope="module")
def written(built, tmp_path_factory):
    path = tmp_path_factory.mktemp("ds") / "demos.hdf5"
    with cameras.CameraRig(built.model, width=32, height=24) as rig:
        rec = recorder.EpisodeRecorder(log_every=2, rig=rig, camera_hz=20.0)
        infos = _feed(rec, built, 40)
    with recorder.DatasetWriter(path, env_meta={"env_name": "test"}) as w:
        key = w.write(rec, built.task, _result(), seed=[0, 1])
    return path, key, infos


def test_every_obs_key_has_num_samples_rows(written):
    path, key, _ = written
    with h5py.File(path) as f:
        demo = f[f"data/{key}"]
        n = demo.attrs["num_samples"]
        assert n == 20  # 40 steps, every 2nd logged
        for name, ds in demo["obs"].items():
            assert ds.shape[0] == n, name
        assert demo["actions"].shape == (n, 7)
        assert demo["states"].shape[0] == n
        assert f["data"].attrs["total"] == n


def test_tactile_rows_match_step_info_without_pad_swap(written):
    path, key, infos = written
    logged = infos[::2]
    with h5py.File(path) as f:
        obs = f[f"data/{key}/obs"]
        np.testing.assert_array_equal(obs["tactile_left"][3], logged[3].tactile[0])
        np.testing.assert_array_equal(obs["tactile_right"][3], logged[3].tactile[1])


def test_camera_frames_at_camera_rate_with_matching_step_index(written):
    path, key, _ = written
    with h5py.File(path) as f:
        img = f[f"data/{key}/images"]
        t_obs = f[f"data/{key}/obs/t"][:]
        t_img = img["t"][:]
        assert img["scene_cam_rgb"].shape == (len(t_img), 24, 32, 3)
        assert img["wrist_cam_depth"].dtype == np.uint16
        np.testing.assert_allclose(np.diff(t_img), 1 / 20.0, atol=DT * 2 + 1e-9)
        np.testing.assert_array_equal(t_obs[img["step_index"][:]], t_img)


def test_task_attributes_recorded(written, built):
    path, key, _ = written
    with h5py.File(path) as f:
        a = f[f"data/{key}"].attrs
        assert a["obj_kind"] == built.task.obj.kind
        assert a["obj_mass"] == pytest.approx(built.task.obj.mass)
        assert bool(a["success"]) is True


def test_existing_dataset_is_not_overwritten(written):
    path, _, _ = written
    with pytest.raises(FileExistsError):
        recorder.DatasetWriter(path, env_meta={})


def test_empty_recorder_is_rejected(built, tmp_path):
    with recorder.DatasetWriter(tmp_path / "x.hdf5", env_meta={}) as w, \
            pytest.raises(ValueError, match="empty"):
        w.write(recorder.EpisodeRecorder(), built.task, _result(), seed=[0])


def test_num_samples_counts_only_logged_steps(built):
    rec = recorder.EpisodeRecorder(log_every=3)
    _feed(rec, built, 10)
    assert rec.num_samples == 4  # steps 0, 3, 6, 9


def test_close_leaves_a_readable_file_and_is_idempotent(built, tmp_path):
    rec = recorder.EpisodeRecorder()
    _feed(rec, built, 3)
    w = recorder.DatasetWriter(tmp_path / "c.hdf5", env_meta={})
    w.write(rec, built.task, _result(), seed=[0])
    w.close()
    w.close()
    with h5py.File(tmp_path / "c.hdf5") as f:
        assert f["data/demo_0"].attrs["num_samples"] == 3


def test_log_every_of_one_is_accepted():
    assert recorder.EpisodeRecorder(log_every=1).num_samples == 0


def test_zero_log_every_is_rejected():
    with pytest.raises(ValueError, match="log_every"):
        recorder.EpisodeRecorder(log_every=0)
