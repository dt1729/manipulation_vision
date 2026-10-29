"""Offscreen RGB + metric depth from named MuJoCo cameras.

Contract: docs/contracts/manipulation_vision.tactile_pnp.cameras.md

Needs a GL context: set ``MUJOCO_GL=egl`` before ``mujoco`` is imported when headless.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import mujoco
import numpy as np

from manipulation_vision.tactile_pnp import mjutil


@dataclass(frozen=True)
class Frame:
    rgb: np.ndarray  # (H, W, 3) uint8
    depth: np.ndarray  # (H, W) float32, metres


class CameraRig:
    """One offscreen renderer shared by several cameras of one model. Call ``close()``."""

    def __init__(self, model: mujoco.MjModel,
                 cameras: Sequence[str] = ("scene_cam", "wrist_cam"),
                 width: int = 128, height: int = 128) -> None:
        buf_w, buf_h = model.vis.global_.offwidth, model.vis.global_.offheight
        if not (0 < width <= buf_w and 0 < height <= buf_h):
            raise ValueError(f"camera size {width}x{height} must be within the offscreen "
                             f"buffer {buf_w}x{buf_h}")
        self._ids = {name: mjutil.camera_id(model, name) for name in cameras}
        self._renderer: mujoco.Renderer | None = mujoco.Renderer(model, height, width)

    def capture(self, data: mujoco.MjData) -> dict[str, Frame]:
        r = self._renderer
        if r is None:
            raise RuntimeError("CameraRig used after close()")
        out = {}
        for name, cam in self._ids.items():
            r.update_scene(data, camera=cam)
            rgb = r.render().copy()
            r.enable_depth_rendering()
            depth = r.render().astype(np.float32)
            r.disable_depth_rendering()
            out[name] = Frame(rgb=rgb, depth=depth)
        return out

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __enter__(self) -> CameraRig:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
