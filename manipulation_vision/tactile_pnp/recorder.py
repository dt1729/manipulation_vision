"""Robomimic-style HDF5 dataset of pick-and-place episodes with tactile and camera streams.

Contract: docs/contracts/manipulation_vision.tactile_pnp.recorder.md

``EpisodeRecorder`` is a ``pnp.Observer`` that buffers one episode in memory;
``DatasetWriter`` appends it to ``data/demo_<k>`` and flushes.
"""

from __future__ import annotations

import dataclasses
import json
import logging
from pathlib import Path
from types import TracebackType

import h5py
import mujoco
import numpy as np

from manipulation_vision.tactile_pnp import tactile
from manipulation_vision.tactile_pnp.cameras import CameraRig
from manipulation_vision.tactile_pnp.pnp import EpisodeResult, Phase, StepInfo
from manipulation_vision.tactile_pnp.scene import Scene, TaskSpec

log = logging.getLogger(__name__)

_GZIP = {"compression": "gzip", "compression_opts": 4}


class EpisodeRecorder:
    """Buffers per-step observations (and lower-rate camera frames) for one episode."""

    def __init__(self, *, log_every: int = 1, rig: CameraRig | None = None,
                 camera_hz: float = 20.0) -> None:
        if log_every < 1:
            raise ValueError(f"log_every must be >= 1, got {log_every}")
        if not camera_hz > 0:
            raise ValueError(f"camera_hz must be > 0, got {camera_hz}")
        self._every = log_every
        self._rig = rig
        self._cam_period = 1.0 / camera_hz
        self._next_cam_t = -np.inf
        self._calls = 0
        self.obs: dict[str, list[np.ndarray]] = {}
        self.actions: list[np.ndarray] = []
        self.states: list[np.ndarray] = []
        self.images: dict[str, list[np.ndarray]] = {}

    @property
    def num_samples(self) -> int:
        return len(self.actions)

    def _append(self, key: str, value: np.ndarray | float | int, store: dict[str, list[np.ndarray]]
                ) -> None:
        store.setdefault(key, []).append(np.asarray(value))

    def __call__(self, scene: Scene, info: StepInfo) -> None:
        self._calls += 1
        if (self._calls - 1) % self._every:
            return
        m, d, h = scene.model, scene.data, scene.handles
        o = self.obs
        self._append("t", info.t, o)
        self._append("phase", np.array(info.phase, dtype=np.int8), o)
        self._append("robot0_joint_pos", d.qpos[h.arm_qpos].copy(), o)
        self._append("robot0_joint_vel", d.qvel[h.arm_dof].copy(), o)
        fq = d.qpos[h.finger_qpos].copy()
        self._append("robot0_gripper_qpos", fq, o)
        self._append("robot0_gripper_width", np.array([np.abs(fq).sum()]), o)
        self._append("robot0_eef_pos", d.site_xpos[h.tcp_site].copy(), o)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, d.site_xmat[h.tcp_site])
        self._append("robot0_eef_quat", quat, o)
        self._append("tactile_left", info.tactile[0].astype(np.float32), o)
        self._append("tactile_right", info.tactile[1].astype(np.float32), o)
        self._append("tactile_active", tactile.active_cells(info.tactile).astype(np.int16), o)
        self._append("contact_force",
                     tactile.finger_contact_forces(m, d, h.finger_bodies, h.obj_geom), o)
        self._append("object_pos", d.qpos[h.obj_qpos:h.obj_qpos + 3].copy(), o)
        self._append("object_quat", d.qpos[h.obj_qpos + 3:h.obj_qpos + 7].copy(), o)
        self.actions.append(np.asarray(info.action, dtype=np.float64))
        self.states.append(np.concatenate([d.qpos, d.qvel]))
        if self._rig is not None and info.t >= self._next_cam_t - 1e-9:
            self._next_cam_t = (info.t if np.isinf(self._next_cam_t)
                                else self._next_cam_t) + self._cam_period
            self._append("t", info.t, self.images)
            self._append("step_index", self.num_samples - 1, self.images)
            for name, frame in self._rig.capture(d).items():
                self._append(f"{name}_rgb", frame.rgb, self.images)
                depth_mm = np.clip(frame.depth * 1000.0, 0, np.iinfo(np.uint16).max)
                self._append(f"{name}_depth", depth_mm.astype(np.uint16), self.images)


def _task_attrs(task: TaskSpec) -> dict[str, object]:
    o = task.obj
    return {
        "task": json.dumps(dataclasses.asdict(task)),
        "obj_kind": o.kind,
        "obj_mass": o.mass,
        "obj_friction": o.friction,
        "obj_half_extents": np.asarray(o.half_extents),
        "obj_pos": np.asarray(o.pos),
        "obj_yaw": o.yaw,
        "place_xy": np.asarray(task.place_xy),
    }


class DatasetWriter:
    """Appends episodes to ``path`` (a new file) as ``data/demo_<k>``."""

    def __init__(self, path: Path, *, env_meta: dict[str, object]) -> None:
        path = Path(path)
        if path.exists():
            raise FileExistsError(f"refusing to overwrite dataset {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._f = h5py.File(path, "w")
        self._data = self._f.create_group("data")
        self._data.attrs.update({
            "total": 0,
            "env_args": json.dumps(env_meta),
            "phase_names": json.dumps([p.name for p in Phase]),
            "tactile_unit": "N",
            "depth_unit": "mm",
            "quat_order": "wxyz",
        })
        self._count = 0

    def write(self, rec: EpisodeRecorder, task: TaskSpec, result: EpisodeResult, *,
              seed: list[int]) -> str:
        n = rec.num_samples
        if n == 0:
            raise ValueError("cannot write an empty recorder (no steps logged)")
        key = f"demo_{self._count}"
        g = self._data.create_group(key)
        g.create_dataset("actions", data=np.stack(rec.actions), **_GZIP)
        g.create_dataset("states", data=np.stack(rec.states), **_GZIP)
        rewards = np.zeros(n)
        rewards[-1] = float(result.success)
        dones = np.zeros(n, dtype=np.int64)
        dones[-1] = 1
        g.create_dataset("rewards", data=rewards)
        g.create_dataset("dones", data=dones)
        obs = g.create_group("obs")
        for k, v in rec.obs.items():
            obs.create_dataset(k, data=np.stack(v), **_GZIP)
        if rec.images:
            img = g.create_group("images")
            for k, v in rec.images.items():
                arr = np.stack(v)
                chunks = (1, *arr.shape[1:]) if arr.ndim > 1 else None
                img.create_dataset(k, data=arr, chunks=chunks, **_GZIP)
        g.attrs.update({
            "num_samples": n,
            "success": bool(result.success),
            "failure": result.failure or "",
            "place_error": result.place_error,
            "sim_time": result.sim_time,
            "grasp_force": np.asarray(result.grasp_force),
            "peak_active": json.dumps(result.peak_active),
            "seed": np.asarray(seed),
            **_task_attrs(task),
        })
        self._data.attrs["total"] = int(self._data.attrs["total"]) + n
        self._count += 1
        self._f.flush()
        log.debug("demo.written key=%s samples=%d frames=%d success=%s", key, n,
                  len(rec.images.get("t", [])), result.success)
        return key

    def close(self) -> None:
        if self._f.id.valid:
            self._f.close()

    def __enter__(self) -> DatasetWriter:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None,
                 tb: TracebackType | None) -> None:
        self.close()
