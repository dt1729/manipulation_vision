import json

import h5py
import pytest

from manipulation_vision.tactile_pnp import cli


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory):
    out = tmp_path_factory.mktemp("runs")
    code = cli.main(["--episodes", "1", "--seed", "3", "--out", str(out), "--run-name", "r",
                     "--cam-size", "64", "48"])
    assert code == 0
    return out / "r"


def test_one_episode_writes_robomimic_layout(run_dir):
    with h5py.File(run_dir / "demos.hdf5") as f:
        demo = f["data/demo_0"]
        n = demo.attrs["num_samples"]
        assert n > 100
        for key in ("actions", "rewards", "dones", "states"):
            assert demo[key].shape[0] == n, key
        for key in ("robot0_joint_pos", "robot0_eef_pos", "tactile_left", "tactile_right",
                    "object_pos", "phase"):
            assert demo[f"obs/{key}"].shape[0] == n, key
        assert demo["obs/tactile_left"].shape[1:] == (16, 16)
        assert json.loads(f["data"].attrs["env_args"])["env_name"] == "PiperTactilePnP"


def test_one_episode_records_camera_frames(run_dir):
    with h5py.File(run_dir / "demos.hdf5") as f:
        img = f["data/demo_0/images"]
        assert img["scene_cam_rgb"].shape[1:] == (48, 64, 3)
        assert img["scene_cam_rgb"].shape[0] == img["wrist_cam_depth"].shape[0] > 10


def test_one_episode_writes_summary_matching_dataset(run_dir):
    s = json.loads((run_dir / "summary.json").read_text())
    with h5py.File(run_dir / "demos.hdf5") as f:
        assert s["successes"] == int(f["data/demo_0"].attrs["success"])
    assert s["episodes"] == 1


def test_no_camera_writes_no_images(tmp_path):
    assert cli.main(["--episodes", "1", "--seed", "3", "--out", str(tmp_path), "--run-name",
                     "nc", "--no-camera"]) == 0
    with h5py.File(tmp_path / "nc" / "demos.hdf5") as f:
        assert "images" not in f["data/demo_0"]


def test_collect_returns_summary_consistent_with_its_dataset(tmp_path):
    s: cli.RunSummary = cli.collect(cli.RunConfig(episodes=2, seed=5, out=tmp_path,
                                                  run_name="c", cameras=False, log_hz=50.0))
    assert s.episodes == 2
    assert s.successes + sum(s.failures.values()) == 2
    assert s.realtime_factor > 0 and s.dataset_bytes == s.dataset.stat().st_size
    with h5py.File(s.dataset) as f:
        assert len(f["data"]) == 2
        assert sum(bool(f[f"data/{k}"].attrs["success"]) for k in f["data"]) == s.successes
        assert "GRASP" in s.peak_active


def test_zero_episodes_is_rejected(tmp_path):
    with pytest.raises(SystemExit) as e:
        cli.main(["--episodes", "0", "--out", str(tmp_path)])
    assert e.value.code == 2


def test_log_rate_not_dividing_control_rate_is_rejected(tmp_path):
    with pytest.raises(SystemExit) as e:
        cli.main(["--episodes", "1", "--out", str(tmp_path), "--log-hz", "30"])
    assert e.value.code == 2
