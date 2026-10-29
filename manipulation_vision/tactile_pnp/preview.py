"""MP4 preview of an episode: scene + wrist camera beside both tactile heatmaps.

Contract: docs/contracts/manipulation_vision.tactile_pnp.preview.md
"""

from __future__ import annotations

from pathlib import Path
from types import TracebackType
from typing import Any

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np

from manipulation_vision.tactile_pnp import tactile
from manipulation_vision.tactile_pnp.pnp import StepInfo
from manipulation_vision.tactile_pnp.scene import Scene

_TEXT = (255, 255, 120)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def heatmap(grid: np.ndarray, vmax: float, cell_px: int = 10) -> np.ndarray:
    """RGB uint8 image of a 2-D taxel grid (N), inferno colormap, ``vmax`` saturates."""
    if grid.ndim != 2:
        raise ValueError(f"grid must be 2-D, got shape {grid.shape}")
    vmax = max(vmax, 1e-6)
    u8 = np.clip(np.asarray(grid, float) / vmax * 255.0, 0, 255).astype(np.uint8)
    big = cv2.resize(u8, (grid.shape[1] * cell_px, grid.shape[0] * cell_px),
                     interpolation=cv2.INTER_NEAREST)
    return cv2.applyColorMap(big, cv2.COLORMAP_INFERNO)[:, :, ::-1].copy()


def _label(img: np.ndarray, text: str, y: int = 16) -> np.ndarray:
    cv2.putText(img, text, (5, y), _FONT, 0.45, _TEXT, 1, cv2.LINE_AA)
    return img


def _pad_to(img: np.ndarray, h: int, w: int) -> np.ndarray:
    out = np.zeros((h, w, 3), np.uint8)
    out[: img.shape[0], : img.shape[1]] = img
    return out


class PreviewWriter:
    """Observer writing an annotated MP4; call ``close()`` to finalise the file."""

    def __init__(self, model: mujoco.MjModel, path: Path, *, fps: int = 30,
                 control_hz: float = 100.0, size: tuple[int, int] = (480, 360)) -> None:
        if not (0 < fps <= control_hz):
            raise ValueError(f"fps must be in (0, control_hz={control_hz}], got {fps}")
        self._every = max(1, round(control_hz / fps))
        self._w, self._h = size
        self._scene_r: mujoco.Renderer | None = mujoco.Renderer(model, self._h, self._w)
        self._wrist_r: mujoco.Renderer | None = mujoco.Renderer(model, self._h // 2,
                                                                self._w // 2)
        self._writer: Any = imageio.get_writer(  # imageio's Writer is untyped
            str(path), fps=fps, quality=8, macro_block_size=1)
        self._calls = 0
        self._peak = np.full(2, 0.05)
        self.frames = 0

    def __call__(self, scene: Scene, info: StepInfo) -> None:
        self._calls += 1
        if (self._calls - 1) % self._every or self._scene_r is None or self._wrist_r is None \
                or self._writer is None:
            return
        d, h = scene.data, scene.handles
        self._scene_r.update_scene(d, camera=h.scene_cam)
        main = _label(self._scene_r.render().copy(),
                      f"t={info.t:5.2f}s  {info.phase.name}")
        self._wrist_r.update_scene(d, camera=h.wrist_cam)
        wrist = _label(self._wrist_r.render().copy(), "wrist_cam")
        self._peak = np.maximum(self._peak, info.tactile.reshape(2, -1).max(axis=1))
        active = tactile.active_cells(info.tactile)
        cell = max(4, (self._h // 2 - 20) // info.tactile.shape[1])
        pads = []
        for k, name in enumerate(("left", "right")):
            hm = heatmap(info.tactile[k], float(self._peak[k]), cell)
            head = _label(np.zeros((20, hm.shape[1], 3), np.uint8),
                          f"{name} {active[k]} on, max {self._peak[k]:.2f}N", 14)
            pads.append(np.vstack([head, hm]))
        pad_row = np.hstack([pads[0], np.zeros((pads[0].shape[0], 4, 3), np.uint8), pads[1]])
        right_w = max(wrist.shape[1], pad_row.shape[1])
        right = np.vstack([_pad_to(wrist, wrist.shape[0], right_w),
                           _pad_to(pad_row, pad_row.shape[0], right_w)])
        height = max(main.shape[0], right.shape[0])
        frame = np.hstack([_pad_to(main, height, main.shape[1]),
                           _pad_to(right, height, right_w)])
        frame = _pad_to(frame, frame.shape[0] + frame.shape[0] % 2,
                        frame.shape[1] + frame.shape[1] % 2)
        self._writer.append_data(frame)
        self.frames += 1

    def close(self) -> None:
        for r in (self._scene_r, self._wrist_r):
            if r is not None:
                r.close()
        self._scene_r = self._wrist_r = None
        if self._writer is not None:
            self._writer.close()
            self._writer = None

    def __enter__(self) -> PreviewWriter:
        return self

    def __exit__(self, exc_type: type[BaseException] | None, exc: BaseException | None,
                 tb: TracebackType | None) -> None:
        self.close()
