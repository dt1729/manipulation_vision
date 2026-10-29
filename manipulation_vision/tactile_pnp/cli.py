"""Seeded, headless pick-and-place data collection: the composition root.

Contract: docs/contracts/manipulation_vision.tactile_pnp.cli.md

    python -m manipulation_vision.tactile_pnp --episodes 20 --seed 0 \\
        --out /data/datasets/tactile_pnp
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")  # before mujoco is imported anywhere

import argparse
import contextlib
import dataclasses
import json
import logging
import time
from collections import Counter
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from manipulation_vision.tactile_pnp import pnp
from manipulation_vision.tactile_pnp.cameras import CameraRig
from manipulation_vision.tactile_pnp.preview import PreviewWriter
from manipulation_vision.tactile_pnp.recorder import DatasetWriter, EpisodeRecorder
from manipulation_vision.tactile_pnp.scene import build_scene, sample_task

log = logging.getLogger("manipulation_vision.tactile_pnp.cli")

DEFAULT_OUT = Path("/data/datasets/tactile_pnp")
ENV_NAME = "PiperTactilePnP"


@dataclasses.dataclass(frozen=True)
class RunConfig:
    episodes: int
    seed: int
    out: Path
    run_name: str | None = None
    cameras: bool = True
    log_hz: float = 100.0
    camera_hz: float = 20.0
    cam_size: tuple[int, int] = (128, 128)
    preview: Path | None = None
    preview_episode: int = 0


@dataclasses.dataclass(frozen=True)
class RunSummary:
    run_dir: Path
    dataset: Path
    episodes: int
    successes: int
    failures: dict[str, int]
    mean_wall_s: float
    realtime_factor: float
    dataset_bytes: int
    peak_active: dict[str, dict[str, float]]


def _episode_rng(seed: int, i: int) -> np.random.Generator:
    return np.random.default_rng([seed, i])


def _offscreen(cfg: RunConfig) -> tuple[int, int]:
    return max(640, cfg.cam_size[0]), max(480, cfg.cam_size[1])


def _peak_summary(results: list[pnp.EpisodeResult]) -> dict[str, dict[str, float]]:
    """Per phase: mean and max over episodes of the episode's peak active cells (max pad)."""
    out = {}
    for phase in pnp.Phase:
        vals = [max(r.peak_active[phase.name]) for r in results if phase.name in r.peak_active]
        if vals:
            out[phase.name] = {"mean": float(np.mean(vals)), "max": float(np.max(vals))}
    return out


def _render_preview(cfg: RunConfig, pnp_cfg: pnp.PnpConfig) -> None:
    assert cfg.preview is not None
    task = sample_task(_episode_rng(cfg.seed, cfg.preview_episode))
    scene = build_scene(task, offscreen=_offscreen(cfg))
    with PreviewWriter(scene.model, cfg.preview, control_hz=pnp_cfg.control_hz) as pw:
        pnp.run_episode(scene, pnp_cfg, observers=[pw])
    log.info("preview.written path=%s frames=%d", cfg.preview, pw.frames)


def collect(cfg: RunConfig) -> RunSummary:
    pnp_cfg = pnp.PnpConfig()
    log_every = round(pnp_cfg.control_hz / cfg.log_hz)
    name = cfg.run_name or f"run_{datetime.now(UTC):%Y%m%d_%H%M%S}_s{cfg.seed}"
    run_dir = cfg.out / name
    dataset = run_dir / "demos.hdf5"
    env_meta: dict[str, object] = {
        "env_name": ENV_NAME, "type": "mujoco_standalone",
        "env_kwargs": {"pnp": dataclasses.asdict(pnp_cfg), "log_hz": cfg.log_hz,
                       "camera_hz": cfg.camera_hz if cfg.cameras else None,
                       "cam_size": list(cfg.cam_size), "seed": cfg.seed},
    }
    log.info("run.start dir=%s episodes=%d seed=%d cameras=%s", run_dir, cfg.episodes,
             cfg.seed, cfg.cameras)
    results: list[pnp.EpisodeResult] = []
    walls: list[float] = []
    with DatasetWriter(dataset, env_meta=env_meta) as writer:
        for i in range(cfg.episodes):
            t0 = time.perf_counter()
            task = sample_task(_episode_rng(cfg.seed, i))
            scene = build_scene(task, offscreen=_offscreen(cfg))
            with contextlib.ExitStack() as stack:
                rig = (stack.enter_context(CameraRig(scene.model, width=cfg.cam_size[0],
                                                     height=cfg.cam_size[1]))
                       if cfg.cameras else None)
                rec = EpisodeRecorder(log_every=log_every, rig=rig, camera_hz=cfg.camera_hz)
                result = pnp.run_episode(scene, pnp_cfg, observers=[rec])
            writer.write(rec, task, result, seed=[cfg.seed, i])
            walls.append(time.perf_counter() - t0)
            results.append(result)
            log.info("episode %d/%d kind=%s success=%s failure=%s wall=%.2fs sim=%.2fs",
                     i + 1, cfg.episodes, task.obj.kind, result.success, result.failure,
                     walls[-1], result.sim_time)
    if cfg.preview is not None:
        _render_preview(cfg, pnp_cfg)
    summary = RunSummary(
        run_dir=run_dir, dataset=dataset, episodes=cfg.episodes,
        successes=sum(r.success for r in results),
        failures=dict(Counter(r.failure for r in results if r.failure)),
        mean_wall_s=float(np.mean(walls)),
        realtime_factor=sum(r.sim_time for r in results) / sum(walls),
        dataset_bytes=dataset.stat().st_size,
        peak_active=_peak_summary(results),
    )
    (run_dir / "summary.json").write_text(
        json.dumps(dataclasses.asdict(summary), indent=2, default=str))
    log.info("run.done success=%d/%d realtime=%.2fx size=%.1fMB", summary.successes,
             summary.episodes, summary.realtime_factor, summary.dataset_bytes / 1e6)
    return summary


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m manipulation_vision.tactile_pnp",
                                description=__doc__.splitlines()[0])
    p.add_argument("--episodes", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument("--run-name", default=None)
    p.add_argument("--no-camera", action="store_true", help="skip RGB-D rendering")
    p.add_argument("--log-hz", type=float, default=100.0, help="must divide 100 (control)")
    p.add_argument("--camera-hz", type=float, default=20.0)
    p.add_argument("--cam-size", type=int, nargs=2, default=(128, 128), metavar=("W", "H"))
    p.add_argument("--preview", type=Path, default=None, help="also render an MP4 of one episode")
    p.add_argument("--preview-episode", type=int, default=0,
                   help="episode index (same seed) to render for --preview")
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def _config(args: argparse.Namespace, p: argparse.ArgumentParser) -> RunConfig:
    control_hz = pnp.PnpConfig().control_hz
    if args.episodes < 1:
        p.error(f"--episodes must be >= 1, got {args.episodes}")
    ratio = control_hz / args.log_hz if args.log_hz > 0 else 0.0
    if ratio < 1 or abs(ratio - round(ratio)) > 1e-9:
        p.error(f"--log-hz must divide the {control_hz:.0f} Hz control rate, got {args.log_hz}")
    if not 0 < args.camera_hz <= args.log_hz:
        p.error(f"--camera-hz must be in (0, log-hz], got {args.camera_hz}")
    return RunConfig(episodes=args.episodes, seed=args.seed, out=args.out,
                     run_name=args.run_name, cameras=not args.no_camera, log_hz=args.log_hz,
                     camera_hz=args.camera_hz, cam_size=tuple(args.cam_size),
                     preview=args.preview, preview_episode=args.preview_episode)


def main(argv: Sequence[str] | None = None) -> int:
    p = _parser()
    args = p.parse_args(argv)
    cfg = _config(args, p)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    s = collect(cfg)
    print(json.dumps(dataclasses.asdict(s), indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
