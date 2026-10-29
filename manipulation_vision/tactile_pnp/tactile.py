"""Per-pad touch measurements from the 2x16x16 ``<touch>`` grid and finger contacts.

Contract: docs/contracts/manipulation_vision.tactile_pnp.tactile.md

Grid layout: ``(pad, row, col)`` with pad 0 = left (link7), 1 = right (link8); values in N.
Touch sites overlap, so a grid sum over-counts force: use ``finger_contact_forces`` for the
physical normal force.
"""

from __future__ import annotations

import mujoco
import numpy as np

ACTIVE_THRESHOLD = 0.05  # N per cell


def read_grids(data: mujoco.MjData, adr: np.ndarray) -> np.ndarray:
    """Current touch readings, float32 ``(2, ny, nx)`` in N (a new array)."""
    return np.asarray(data.sensordata[adr], dtype=np.float32)


def _check(grids: np.ndarray, threshold: float) -> None:
    if grids.ndim != 3 or grids.shape[0] != 2:
        raise ValueError(f"grids must have shape (2, ny, nx), got shape {grids.shape}")
    if threshold < 0:
        raise ValueError(f"threshold must be >= 0, got {threshold}")


def active_cells(grids: np.ndarray, threshold: float = ACTIVE_THRESHOLD) -> np.ndarray:
    """Cells strictly above ``threshold`` per pad, int ``(2,)``."""
    _check(grids, threshold)
    return (grids > threshold).reshape(2, -1).sum(axis=1)


def pad_contact(grids: np.ndarray, *, min_cells: int = 1,
                threshold: float = ACTIVE_THRESHOLD) -> np.ndarray:
    """Bool ``(2,)``: pad has at least ``min_cells`` active cells."""
    if min_cells < 1:
        raise ValueError(f"min_cells must be >= 1, got {min_cells}")
    return active_cells(grids, threshold) >= min_cells


def finger_contact_forces(model: mujoco.MjModel, data: mujoco.MjData,
                          finger_bodies: tuple[int, int], other_geom: int) -> np.ndarray:
    """Summed contact-normal force (N) between any geom of each finger body and
    ``other_geom``, ``(2,)``."""
    out = np.zeros(2)
    wrench = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if other_geom == g1:
            finger_geom = g2
        elif other_geom == g2:
            finger_geom = g1
        else:
            continue
        body = int(model.geom_bodyid[finger_geom])
        for k, b in enumerate(finger_bodies):
            if body == b:
                mujoco.mj_contactForce(model, data, i, wrench)
                out[k] += abs(wrench[0])
    return out
