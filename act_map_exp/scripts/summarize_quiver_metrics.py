#!/usr/bin/env python3
"""Summarize per-iteration quiver metrics (alignment + visibility) and save next to results.

Input: quiver file with blocks separated by blank lines.
Each line: x,y,z,dx,dy,dz (extra columns are ignored).
Outputs (next to quiver file):
  - <prefix>_pose_metrics_progress.csv
  - <prefix>_pose_metrics_per_iter_summary.csv
  - <prefix>_pose_metrics_per_pose_summary.csv
  - <prefix>_metrics_progress.png (if matplotlib is available)

If no variation is specified, metrics are combined across all variations
under the root folder and saved once in the parent folder.

This script re-evaluates metrics using the quiver directions and a point cloud,
without touching optimization.

Alignment metrics:
  u = Kᵀ c (dot between view dir and point direction)
  - mean(u)
  - mean(max(u, 0))
  - normalized alignment = (mean(u) + 1) / 2 in [0, 1]

Visibility metrics:
  1) Schedule-based (per iteration alpha):
     - vis_sched_count = count(u >= cos(alpha_i))
     - vis_sched_score = sum(sigmoid(ks_i * (u - cos(alpha_i))))
  2) Fixed 30-deg FOV (half-alpha = 15 deg):
     - vis30_count = count(u >= cos(15))
     - vis30_score = sum(sigmoid(ks * (u - cos(15))))

Schedule alpha is resolved from --fov-schedule or FOV_OPT_FOV_SCHEDULE,
otherwise falls back to the optimizer's default schedule (180/90/60/vis_angle).
"""

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import numpy as np


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "y", "on")


def parse_schedule(text: str) -> List[float]:
    if not text:
        return []
    parts = [p.strip() for p in text.split(",") if p.strip()]
    out: List[float] = []
    for p in parts:
        try:
            out.append(float(p))
        except ValueError:
            continue
    return out


def build_alpha_schedule(n_iter: int,
                         schedule_deg: List[float],
                         vis_angle_deg: float) -> np.ndarray:
    alpha = np.zeros((n_iter,), dtype=float)
    if schedule_deg:
        stages = len(schedule_deg)
        stage_len = max(1, n_iter // stages)
        for i in range(n_iter):
            stage_idx = min(i // stage_len, stages - 1)
            alpha[i] = float(schedule_deg[stage_idx])
        return alpha
    for i in range(n_iter):
        ratio = float(i) / max(1, n_iter)
        if ratio < 0.05:
            alpha[i] = 179.0
        elif ratio < 0.10:
            alpha[i] = 90.0
        elif ratio < 0.15:
            alpha[i] = 60.0
        else:
            alpha[i] = float(vis_angle_deg)
    return alpha


def build_ks_schedule(alpha_deg: np.ndarray,
                      ks_base: float,
                      ks_from_visibility: bool,
                      ks_transition_deg: float) -> np.ndarray:
    if not ks_from_visibility or ks_transition_deg <= 0.0:
        return np.full_like(alpha_deg, float(ks_base), dtype=float)
    out = np.zeros_like(alpha_deg, dtype=float)
    delta_rad = float(np.deg2rad(ks_transition_deg))
    for i, a in enumerate(alpha_deg):
        sin_a = float(np.sin(np.deg2rad(a)))
        if abs(sin_a) < 1e-6 or delta_rad <= 0.0:
            out[i] = float(ks_base)
            continue
        dyn = 2.94 / (delta_rad * sin_a)
        if not np.isfinite(dyn) or dyn <= 0.0:
            out[i] = float(ks_base)
        else:
            out[i] = float(dyn)
    return out


def normalized_progress(cur: np.ndarray,
                        base: np.ndarray,
                        upper: float = 1.0,
                        eps: float = 1e-6) -> np.ndarray:
    denom = np.maximum(eps, upper - base)
    return (cur - base) / denom


def parse_blocks(path: Path, min_cols: int = 6) -> np.ndarray:
    blocks: List[np.ndarray] = []
    cur: List[List[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                if cur:
                    blocks.append(np.asarray(cur, dtype=float))
                    cur = []
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) < min_cols:
                continue
            cur.append([float(v) for v in parts[:min_cols]])
    if cur:
        blocks.append(np.asarray(cur, dtype=float))

    if not blocks:
        raise RuntimeError(f"No blocks found in {path}")

    min_n_pose = min(len(b) for b in blocks)
    blocks = [b[:min_n_pose] for b in blocks]
    return np.stack(blocks, axis=0)

def append_suffix_before_extension(path: Path, suffix: str) -> Path:
    name = path.name
    if "." in name:
        stem = path.stem
        return path.with_name(f"{stem}{suffix}{path.suffix}")
    return path.with_name(f"{name}{suffix}")


def resolve_metrics_path(quiver_path: Path) -> Optional[Path]:
    if quiver_path.name.endswith("_metrics.txt"):
        return quiver_path if quiver_path.exists() else None
    cand = append_suffix_before_extension(quiver_path, "_metrics")
    if cand.exists():
        return cand
    return None


def resolve_debug_log_path(quiver_path: Path, explicit: str) -> Optional[Path]:
    if explicit:
        path = Path(explicit).expanduser().resolve()
        return path if path.exists() else None
    cand = quiver_path.parent / "optimization_debug.csv"
    if cand.exists():
        return cand
    return None


def load_debug_arrays(path: Optional[Path],
                      n_iter: int,
                      n_pose: int) -> Tuple[List[str], Dict[str, np.ndarray]]:
    if path is None or not path.exists():
        return [], {}
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return [], {}
        fields = [f for f in reader.fieldnames if f not in ("iter", "pose_idx")]
        arrays = {f: np.full((n_iter, n_pose), np.nan, dtype=float) for f in fields}
        for row in reader:
            try:
                iter_idx = int(float(row.get("iter", "")))
                pose_idx = int(float(row.get("pose_idx", ""))) - 1
            except (TypeError, ValueError):
                continue
            if iter_idx < 0 or pose_idx < 0:
                continue
            if iter_idx >= n_iter or pose_idx >= n_pose:
                continue
            for field in fields:
                raw = row.get(field, "")
                if raw is None or raw == "":
                    continue
                try:
                    arrays[field][iter_idx, pose_idx] = float(raw)
                except ValueError:
                    continue
    return fields, arrays


def parse_cols(cols: str) -> Tuple[int, int, int]:
    parts = [p.strip() for p in cols.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError("--points-cols must be 3 comma-separated integers")
    return int(parts[0]), int(parts[1]), int(parts[2])


def load_points(path: Path, cols: Tuple[int, int, int], stride: int, max_points: int) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Points file does not exist: {path}")
    pts: List[List[float]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) <= max(cols):
                continue
            try:
                pts.append([float(parts[cols[0]]), float(parts[cols[1]]), float(parts[cols[2]])])
            except ValueError:
                continue
    if not pts:
        return np.empty((0, 3), dtype=float)
    arr = np.asarray(pts, dtype=float)
    if stride > 1:
        arr = arr[::stride]
    if max_points > 0 and arr.shape[0] > max_points:
        rng = np.random.default_rng(7)
        idx = rng.choice(arr.shape[0], size=max_points, replace=False)
        arr = arr[idx]
    return arr


def compute_metrics(pos: np.ndarray,
                    dirs: np.ndarray,
                    points: np.ndarray,
                    alpha_deg: np.ndarray,
                    ks_iter: np.ndarray,
                    vis30_deg: float,
                    vis30_ks: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    n_iter, n_pose, _ = pos.shape
    align_mean = np.zeros((n_iter, n_pose), dtype=float)
    align_pos_mean = np.zeros((n_iter, n_pose), dtype=float)
    vis_sched_count = np.zeros((n_iter, n_pose), dtype=float)
    vis_sched_score = np.zeros((n_iter, n_pose), dtype=float)
    vis30_count = np.zeros((n_iter, n_pose), dtype=float)
    vis30_score = np.zeros((n_iter, n_pose), dtype=float)
    if points.size == 0:
        return align_mean, align_pos_mean, vis_sched_count, vis_sched_score, vis30_count, vis30_score
    cos_alpha_30 = float(np.cos(np.deg2rad(vis30_deg)))
    cos_alpha_sched = np.cos(np.deg2rad(alpha_deg))
    for i in range(n_iter):
        cos_a = float(cos_alpha_sched[i])
        ks_i = float(ks_iter[i])
        for j in range(n_pose):
            p = pos[i, j]
            d = dirs[i, j]
            dn = d / (np.linalg.norm(d) + 1e-9)
            v = points - p[None, :]
            dist = np.linalg.norm(v, axis=1) + 1e-9
            u = (v @ dn) / dist
            align_mean[i, j] = float(np.mean(u))
            align_pos_mean[i, j] = float(np.mean(np.maximum(u, 0.0)))
            vis_sched_count[i, j] = float(np.count_nonzero(u >= cos_a))
            logits_sched = ks_i * (u - cos_a)
            logits_sched = np.clip(logits_sched, -60.0, 60.0)
            vis_sched_score[i, j] = float(np.sum(1.0 / (1.0 + np.exp(-logits_sched))))
            vis30_count[i, j] = float(np.count_nonzero(u >= cos_alpha_30))
            logits30 = vis30_ks * (u - cos_alpha_30)
            logits30 = np.clip(logits30, -60.0, 60.0)
            vis30_score[i, j] = float(np.sum(1.0 / (1.0 + np.exp(-logits30))))
    return align_mean, align_pos_mean, vis_sched_count, vis_sched_score, vis30_count, vis30_score


def write_progress_csv(path: Path,
                       alpha_deg: np.ndarray,
                       align_mean: np.ndarray,
                       align_pos_mean: np.ndarray,
                       align_norm_mean: np.ndarray,
                       align_mean_delta: np.ndarray,
                       align_pos_delta: np.ndarray,
                       align_norm_rel: np.ndarray,
                       vis_sched_count: np.ndarray,
                       vis_sched_score: np.ndarray,
                       vis30_count: np.ndarray,
                       vis30_score: np.ndarray) -> None:
    n_iter, n_pose = align_mean.shape
    with path.open("w", encoding="utf-8") as f:
        f.write(
            "iter,pose_idx,alpha_deg,"
            "align_mean,align_pos_mean,align_norm_mean,"
            "align_mean_delta,align_pos_delta,align_norm_rel,"
            "vis_sched_count,vis_sched_score,vis30_count,vis30_score\n"
        )
        for i in range(n_iter):
            for j in range(n_pose):
                f.write(
                    f"{i},{j},{alpha_deg[i]:.4f},"
                    f"{align_mean[i, j]:.6f},{align_pos_mean[i, j]:.6f},{align_norm_mean[i, j]:.6f},"
                    f"{align_mean_delta[i, j]:.6f},{align_pos_delta[i, j]:.6f},{align_norm_rel[i, j]:.6f},"
                    f"{vis_sched_count[i, j]:.6f},{vis_sched_score[i, j]:.6f},"
                    f"{vis30_count[i, j]:.6f},{vis30_score[i, j]:.6f}\n"
                )


def write_progress_csv_full(path: Path,
                            alpha_deg: np.ndarray,
                            align_mean: np.ndarray,
                            align_pos_mean: np.ndarray,
                            align_norm_mean: np.ndarray,
                            align_mean_delta: np.ndarray,
                            align_pos_delta: np.ndarray,
                            align_norm_rel: np.ndarray,
                            vis_sched_count: np.ndarray,
                            vis_sched_score: np.ndarray,
                            vis30_count: np.ndarray,
                            vis30_score: np.ndarray,
                            debug_fields: List[str],
                            debug_arrays: Dict[str, np.ndarray]) -> None:
    n_iter, n_pose = align_mean.shape
    debug_headers = [f"dbg_{name}" for name in debug_fields]
    with path.open("w", encoding="utf-8") as f:
        f.write(
            "iter,pose_idx,alpha_deg,"
            "align_mean,align_pos_mean,align_norm_mean,"
            "align_mean_delta,align_pos_delta,align_norm_rel,"
            "vis_sched_count,vis_sched_score,vis30_count,vis30_score"
        )
        if debug_headers:
            f.write("," + ",".join(debug_headers))
        f.write("\n")
        for i in range(n_iter):
            for j in range(n_pose):
                row = (
                    f"{i},{j},{alpha_deg[i]:.4f},"
                    f"{align_mean[i, j]:.6f},{align_pos_mean[i, j]:.6f},{align_norm_mean[i, j]:.6f},"
                    f"{align_mean_delta[i, j]:.6f},{align_pos_delta[i, j]:.6f},{align_norm_rel[i, j]:.6f},"
                    f"{vis_sched_count[i, j]:.6f},{vis_sched_score[i, j]:.6f},"
                    f"{vis30_count[i, j]:.6f},{vis30_score[i, j]:.6f}"
                )
                if debug_headers:
                    dbg_vals = []
                    for name in debug_fields:
                        arr = debug_arrays.get(name)
                        if arr is None:
                            dbg_vals.append("")
                        else:
                            val = arr[i, j]
                            if np.isnan(val):
                                dbg_vals.append("")
                            else:
                                dbg_vals.append(f"{val:.6f}")
                    row += "," + ",".join(dbg_vals)
                f.write(row + "\n")


def write_iter_summary(path: Path,
                       alpha_deg: np.ndarray,
                       ks_iter: np.ndarray,
                       align_mean: np.ndarray,
                       align_pos_mean: np.ndarray,
                       align_norm_mean: np.ndarray,
                       align_norm_rel: np.ndarray,
                       align_pos_rel: np.ndarray,
                       vis_sched_count: np.ndarray,
                       vis_sched_score: np.ndarray,
                       vis30_count: np.ndarray,
                       vis30_score: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write(
            "iter,alpha_deg,ks_iter,"
            "align_mean,align_min,align_max,"
            "align_pos_mean,align_pos_min,align_pos_max,"
            "align_norm_mean,align_norm_min,align_norm_max,"
            "align_norm_rel_mean,align_pos_rel_mean,"
            "vis_sched_count_mean,vis_sched_count_min,vis_sched_count_max,"
            "vis_sched_score_mean,vis_sched_score_min,vis_sched_score_max,"
            "vis30_count_mean,vis30_count_min,vis30_count_max,"
            "vis30_score_mean,vis30_score_min,vis30_score_max\n"
        )
        for i in range(align_mean.shape[0]):
            c = align_mean[i]
            s = align_pos_mean[i]
            n = align_norm_mean[i]
            nr = align_norm_rel[i]
            pr = align_pos_rel[i]
            vsc = vis_sched_count[i]
            vss = vis_sched_score[i]
            v30c = vis30_count[i]
            v30s = vis30_score[i]
            f.write(
                f"{i},{alpha_deg[i]:.4f},{ks_iter[i]:.6f},"
                f"{c.mean():.6f},{c.min():.6f},{c.max():.6f},"
                f"{s.mean():.6f},{s.min():.6f},{s.max():.6f},"
                f"{n.mean():.6f},{n.min():.6f},{n.max():.6f},"
                f"{nr.mean():.6f},{pr.mean():.6f},"
                f"{vsc.mean():.6f},{vsc.min():.6f},{vsc.max():.6f},"
                f"{vss.mean():.6f},{vss.min():.6f},{vss.max():.6f},"
                f"{v30c.mean():.6f},{v30c.min():.6f},{v30c.max():.6f},"
                f"{v30s.mean():.6f},{v30s.min():.6f},{v30s.max():.6f}\n"
            )


def write_pose_summary(path: Path,
                       align_mean: np.ndarray,
                       align_pos_mean: np.ndarray,
                       align_norm_mean: np.ndarray,
                       align_norm_rel: np.ndarray,
                       align_pos_rel: np.ndarray,
                       vis_sched_count: np.ndarray,
                       vis_sched_score: np.ndarray,
                       vis30_count: np.ndarray,
                       vis30_score: np.ndarray) -> None:
    first_c = align_mean[0]
    last_c = align_mean[-1]
    first_s = align_pos_mean[0]
    last_s = align_pos_mean[-1]
    first_n = align_norm_mean[0]
    last_n = align_norm_mean[-1]
    last_nr = align_norm_rel[-1]
    last_pr = align_pos_rel[-1]
    first_vsc = vis_sched_count[0]
    last_vsc = vis_sched_count[-1]
    first_vss = vis_sched_score[0]
    last_vss = vis_sched_score[-1]
    first_vc = vis30_count[0]
    last_vc = vis30_count[-1]
    first_vs = vis30_score[0]
    last_vs = vis30_score[-1]
    with path.open("w", encoding="utf-8") as f:
        f.write(
            "pose_idx,align_first,align_last,align_delta,"
            "align_pos_first,align_pos_last,align_pos_delta,"
            "align_norm_first,align_norm_last,align_norm_delta,align_norm_rel_last,align_pos_rel_last,"
            "vis_sched_count_first,vis_sched_count_last,vis_sched_count_delta,"
            "vis_sched_score_first,vis_sched_score_last,vis_sched_score_delta,"
            "vis30_count_first,vis30_count_last,vis30_count_delta,"
            "vis30_score_first,vis30_score_last,vis30_score_delta\n"
        )
        for j in range(align_mean.shape[1]):
            f.write(
                f"{j},{first_c[j]:.6f},{last_c[j]:.6f},{(last_c[j]-first_c[j]):.6f},"
                f"{first_s[j]:.6f},{last_s[j]:.6f},{(last_s[j]-first_s[j]):.6f},"
                f"{first_n[j]:.6f},{last_n[j]:.6f},{(last_n[j]-first_n[j]):.6f},"
                f"{last_nr[j]:.6f},{last_pr[j]:.6f},"
                f"{first_vsc[j]:.6f},{last_vsc[j]:.6f},{(last_vsc[j]-first_vsc[j]):.6f},"
                f"{first_vss[j]:.6f},{last_vss[j]:.6f},{(last_vss[j]-first_vss[j]):.6f},"
                f"{first_vc[j]:.6f},{last_vc[j]:.6f},{(last_vc[j]-first_vc[j]):.6f},"
                f"{first_vs[j]:.6f},{last_vs[j]:.6f},{(last_vs[j]-first_vs[j]):.6f}\n"
            )


def save_plot(path: Path,
              align_mean: np.ndarray,
              align_pos_mean: np.ndarray,
              align_norm_rel: np.ndarray,
              vis_sched_count: np.ndarray,
              vis_sched_score: np.ndarray,
              vis30_count: np.ndarray,
              vis30_score: np.ndarray) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return False

    iters = np.arange(align_mean.shape[0])
    fig, (ax1, ax2) = plt.subplots(nrows=2, figsize=(8, 6.6), sharex=True)
    ax1.plot(iters, align_mean.mean(axis=1), color="#1f77b4", label="align mean")
    ax1.fill_between(iters, align_mean.min(axis=1), align_mean.max(axis=1),
                     color="#1f77b4", alpha=0.15)
    ax1.set_ylabel("align mean")
    ax1.grid(True, alpha=0.2)

    ax1b = ax1.twinx()
    ax1b.plot(iters, align_pos_mean.mean(axis=1), color="#d62728", label="align+ mean")
    ax1b.fill_between(iters, align_pos_mean.min(axis=1), align_pos_mean.max(axis=1),
                      color="#d62728", alpha=0.12)
    ax1b.plot(iters, align_norm_rel.mean(axis=1), color="#111111", ls="--",
              label="align norm rel")
    ax1b.set_ylabel("align+ / norm rel")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1b.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left", frameon=False)

    ax2.plot(iters, vis_sched_count.mean(axis=1), color="#2ca02c", label="vis sched count")
    ax2.fill_between(iters, vis_sched_count.min(axis=1), vis_sched_count.max(axis=1),
                     color="#2ca02c", alpha=0.18)
    ax2.plot(iters, vis30_count.mean(axis=1), color="#2ca02c", ls="--", label="vis30 count")
    ax2.set_ylabel("visibility count")
    ax2.grid(True, alpha=0.2)
    ax2b = ax2.twinx()
    ax2b.plot(iters, vis_sched_score.mean(axis=1), color="#9467bd", label="vis sched score")
    ax2b.fill_between(iters, vis_sched_score.min(axis=1), vis_sched_score.max(axis=1),
                      color="#9467bd", alpha=0.12)
    ax2b.plot(iters, vis30_score.mean(axis=1), color="#9467bd", ls="--", label="vis30 score")
    ax2b.set_ylabel("visibility score")

    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc="upper left", frameon=False)
    ax2.set_xlabel("iteration")

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return True


def save_plot_summary(path: Path,
                      align_mean: np.ndarray,
                      align_min: np.ndarray,
                      align_max: np.ndarray,
                      align_pos_mean: np.ndarray,
                      align_pos_min: np.ndarray,
                      align_pos_max: np.ndarray,
                      align_norm_rel_mean: np.ndarray,
                      vis_sched_count_mean: np.ndarray,
                      vis_sched_count_min: np.ndarray,
                      vis_sched_count_max: np.ndarray,
                      vis_sched_score_mean: np.ndarray,
                      vis_sched_score_min: np.ndarray,
                      vis_sched_score_max: np.ndarray,
                      vis30_count_mean: np.ndarray,
                      vis30_count_min: np.ndarray,
                      vis30_count_max: np.ndarray,
                      vis30_score_mean: np.ndarray,
                      vis30_score_min: np.ndarray,
                      vis30_score_max: np.ndarray) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return False

    iters = np.arange(align_mean.shape[0])
    fig, (ax1, ax2) = plt.subplots(nrows=2, figsize=(8, 6.6), sharex=True)
    ax1.plot(iters, align_mean, color="#1f77b4", label="align mean")
    ax1.fill_between(iters, align_min, align_max, color="#1f77b4", alpha=0.15)
    ax1.set_ylabel("align mean")
    ax1.grid(True, alpha=0.2)

    ax1b = ax1.twinx()
    ax1b.plot(iters, align_pos_mean, color="#d62728", label="align+ mean")
    ax1b.fill_between(iters, align_pos_min, align_pos_max, color="#d62728", alpha=0.12)
    ax1b.plot(iters, align_norm_rel_mean, color="#111111", ls="--", label="align norm rel")
    ax1b.set_ylabel("align+ / norm rel")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1b.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left", frameon=False)

    ax2.plot(iters, vis_sched_count_mean, color="#2ca02c", label="vis sched count")
    ax2.fill_between(iters, vis_sched_count_min, vis_sched_count_max, color="#2ca02c", alpha=0.18)
    ax2.plot(iters, vis30_count_mean, color="#2ca02c", ls="--", label="vis30 count")
    ax2.set_ylabel("visibility count")
    ax2.grid(True, alpha=0.2)
    ax2b = ax2.twinx()
    ax2b.plot(iters, vis_sched_score_mean, color="#9467bd", label="vis sched score")
    ax2b.fill_between(iters, vis_sched_score_min, vis_sched_score_max, color="#9467bd", alpha=0.12)
    ax2b.plot(iters, vis30_score_mean, color="#9467bd", ls="--", label="vis30 score")
    ax2b.set_ylabel("visibility score")

    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc="upper left", frameon=False)
    ax2.set_xlabel("iteration")

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return True


def summarize_combined(runs: List[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                                       np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
                       alpha_deg: np.ndarray,
                       ks_iter: np.ndarray,
                       out_dir: Path,
                       no_plots: bool,
                       tag: str) -> None:

    max_iters = max(r[0].shape[0] for r in runs)
    align_mean = np.zeros((max_iters,), dtype=float)
    align_min = np.zeros((max_iters,), dtype=float)
    align_max = np.zeros((max_iters,), dtype=float)
    align_pos_mean = np.zeros((max_iters,), dtype=float)
    align_pos_min = np.zeros((max_iters,), dtype=float)
    align_pos_max = np.zeros((max_iters,), dtype=float)
    align_norm_mean = np.zeros((max_iters,), dtype=float)
    align_norm_min = np.zeros((max_iters,), dtype=float)
    align_norm_max = np.zeros((max_iters,), dtype=float)
    align_norm_rel_mean = np.zeros((max_iters,), dtype=float)
    align_pos_rel_mean = np.zeros((max_iters,), dtype=float)
    vis_sched_count_mean = np.zeros((max_iters,), dtype=float)
    vis_sched_count_min = np.zeros((max_iters,), dtype=float)
    vis_sched_count_max = np.zeros((max_iters,), dtype=float)
    vis_sched_score_mean = np.zeros((max_iters,), dtype=float)
    vis_sched_score_min = np.zeros((max_iters,), dtype=float)
    vis_sched_score_max = np.zeros((max_iters,), dtype=float)
    vis30_count_mean = np.zeros((max_iters,), dtype=float)
    vis30_count_min = np.zeros((max_iters,), dtype=float)
    vis30_count_max = np.zeros((max_iters,), dtype=float)
    vis30_score_mean = np.zeros((max_iters,), dtype=float)
    vis30_score_min = np.zeros((max_iters,), dtype=float)
    vis30_score_max = np.zeros((max_iters,), dtype=float)
    sample_counts = np.zeros((max_iters,), dtype=int)

    for i in range(max_iters):
        align_vals = []
        align_pos_vals = []
        align_norm_vals = []
        align_norm_rel_vals = []
        align_pos_rel_vals = []
        vis_sched_count_vals = []
        vis_sched_score_vals = []
        vis30_count_vals = []
        vis30_score_vals = []
        for (align, align_pos, align_norm, align_norm_rel, align_pos_rel,
             vis_sched_count, vis_sched_score, vis30_count, vis30_score) in runs:
            if i >= align.shape[0]:
                continue
            align_vals.append(align[i].ravel())
            align_pos_vals.append(align_pos[i].ravel())
            align_norm_vals.append(align_norm[i].ravel())
            align_norm_rel_vals.append(align_norm_rel[i].ravel())
            align_pos_rel_vals.append(align_pos_rel[i].ravel())
            vis_sched_count_vals.append(vis_sched_count[i].ravel())
            vis_sched_score_vals.append(vis_sched_score[i].ravel())
            vis30_count_vals.append(vis30_count[i].ravel())
            vis30_score_vals.append(vis30_score[i].ravel())
        if not align_vals:
            continue
        align_all = np.concatenate(align_vals)
        align_pos_all = np.concatenate(align_pos_vals)
        align_norm_all = np.concatenate(align_norm_vals)
        align_norm_rel_all = np.concatenate(align_norm_rel_vals)
        align_pos_rel_all = np.concatenate(align_pos_rel_vals)
        vis_sched_count_all = np.concatenate(vis_sched_count_vals)
        vis_sched_score_all = np.concatenate(vis_sched_score_vals)
        vis30_count_all = np.concatenate(vis30_count_vals)
        vis30_score_all = np.concatenate(vis30_score_vals)
        align_mean[i] = float(np.mean(align_all))
        align_min[i] = float(np.min(align_all))
        align_max[i] = float(np.max(align_all))
        align_pos_mean[i] = float(np.mean(align_pos_all))
        align_pos_min[i] = float(np.min(align_pos_all))
        align_pos_max[i] = float(np.max(align_pos_all))
        align_norm_mean[i] = float(np.mean(align_norm_all))
        align_norm_min[i] = float(np.min(align_norm_all))
        align_norm_max[i] = float(np.max(align_norm_all))
        align_norm_rel_mean[i] = float(np.mean(align_norm_rel_all))
        align_pos_rel_mean[i] = float(np.mean(align_pos_rel_all))
        vis_sched_count_mean[i] = float(np.mean(vis_sched_count_all))
        vis_sched_count_min[i] = float(np.min(vis_sched_count_all))
        vis_sched_count_max[i] = float(np.max(vis_sched_count_all))
        vis_sched_score_mean[i] = float(np.mean(vis_sched_score_all))
        vis_sched_score_min[i] = float(np.min(vis_sched_score_all))
        vis_sched_score_max[i] = float(np.max(vis_sched_score_all))
        vis30_count_mean[i] = float(np.mean(vis30_count_all))
        vis30_count_min[i] = float(np.min(vis30_count_all))
        vis30_count_max[i] = float(np.max(vis30_count_all))
        vis30_score_mean[i] = float(np.mean(vis30_score_all))
        vis30_score_min[i] = float(np.min(vis30_score_all))
        vis30_score_max[i] = float(np.max(vis30_score_all))
        sample_counts[i] = int(align_all.size)

    out_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = out_dir / f"combined{tag}_pose_metrics_per_iter_summary.csv"
    with summary_csv.open("w", encoding="utf-8") as f:
        f.write(
            "iter,alpha_deg,ks_iter,align_mean,align_min,align_max,"
            "align_pos_mean,align_pos_min,align_pos_max,"
            "align_norm_mean,align_norm_min,align_norm_max,"
            "align_norm_rel_mean,align_pos_rel_mean,"
            "vis_sched_count_mean,vis_sched_count_min,vis_sched_count_max,"
            "vis_sched_score_mean,vis_sched_score_min,vis_sched_score_max,"
            "vis30_count_mean,vis30_count_min,vis30_count_max,"
            "vis30_score_mean,vis30_score_min,vis30_score_max,num_samples\n"
        )
        for i in range(max_iters):
            if sample_counts[i] == 0:
                continue
            alpha = alpha_deg[i] if i < len(alpha_deg) else 0.0
            ks = ks_iter[i] if i < len(ks_iter) else 0.0
            f.write(
                f"{i},{alpha:.4f},{ks:.6f},"
                f"{align_mean[i]:.6f},{align_min[i]:.6f},{align_max[i]:.6f},"
                f"{align_pos_mean[i]:.6f},{align_pos_min[i]:.6f},{align_pos_max[i]:.6f},"
                f"{align_norm_mean[i]:.6f},{align_norm_min[i]:.6f},{align_norm_max[i]:.6f},"
                f"{align_norm_rel_mean[i]:.6f},{align_pos_rel_mean[i]:.6f},"
                f"{vis_sched_count_mean[i]:.6f},{vis_sched_count_min[i]:.6f},{vis_sched_count_max[i]:.6f},"
                f"{vis_sched_score_mean[i]:.6f},{vis_sched_score_min[i]:.6f},{vis_sched_score_max[i]:.6f},"
                f"{vis30_count_mean[i]:.6f},{vis30_count_min[i]:.6f},{vis30_count_max[i]:.6f},"
                f"{vis30_score_mean[i]:.6f},{vis30_score_min[i]:.6f},{vis30_score_max[i]:.6f},"
                f"{sample_counts[i]}\n"
            )

    plot_png = out_dir / f"combined{tag}_metrics_progress.png"
    plotted = False if no_plots else save_plot_summary(
        plot_png,
        align_mean, align_min, align_max,
        align_pos_mean, align_pos_min, align_pos_max,
        align_norm_rel_mean,
        vis_sched_count_mean, vis_sched_count_min, vis_sched_count_max,
        vis_sched_score_mean, vis_sched_score_min, vis_sched_score_max,
        vis30_count_mean, vis30_count_min, vis30_count_max,
        vis30_score_mean, vis30_score_min, vis30_score_max,
    )

    print(f"[combined] runs={len(runs)} iters={max_iters}")
    print(
        f"  align: first mean={align_mean[0]:.3f} last mean={align_mean[max_iters-1]:.3f}"
    )
    print(
        f"  align+: first mean={align_pos_mean[0]:.3f} last mean={align_pos_mean[max_iters-1]:.3f}"
    )
    print(f"  saved: {summary_csv}")
    if plotted:
        print(f"  plot: {plot_png}")
    else:
        print("  plot: skipped (matplotlib not available or --no-plots)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize quiver files and save CSV/PNG next to results."
    )
    parser.add_argument("--root", default="", help="Root directory to search.")
    parser.add_argument("--metrics", nargs="*", default=[], help="Explicit quiver files.")
    parser.add_argument("--variation", default="",
                        help="Filter to a specific variation name (e.g., diagonal).")
    parser.add_argument("--pattern", default="quivers*.txt",
                        help="Glob pattern for quiver files under root.")
    parser.add_argument("--include-metrics", action="store_true",
                        help="Include *_metrics.txt files when scanning.")
    parser.add_argument("--points", default="",
                        help="Point cloud file for fixed-FOV evaluation.")
    parser.add_argument("--points-cols", default="1,2,3",
                        help="Columns for XYZ in points file (0-based).")
    parser.add_argument("--point-stride", type=int, default=1,
                        help="Stride for points subsampling (alignment eval).")
    parser.add_argument("--max-points", type=int, default=0,
                        help="Max points to load (0 = all).")
    parser.add_argument("--vis-ks", type=float, default=_env_float("FOV_OPT_KS", 15.0),
                        help="Sigmoid slope for visibility scores.")
    parser.add_argument("--vis-angle", type=float, default=_env_float("FOV_OPT_VIS_ANGLE_DEG", 15.0),
                        help="Base visibility half-angle (deg) when no schedule is provided.")
    parser.add_argument("--fov-schedule", default=os.environ.get("FOV_OPT_FOV_SCHEDULE", ""),
                        help="Comma-separated FoV schedule (deg).")
    ks_group = parser.add_mutually_exclusive_group()
    ks_group.add_argument("--ks-from-visibility", action="store_true",
                          default=_env_bool("FOV_OPT_KS_FROM_VISIBILITY", False),
                          help="Derive ks from alpha and ks_transition_deg.")
    ks_group.add_argument("--no-ks-from-visibility", action="store_false",
                          dest="ks_from_visibility",
                          help="Disable ks-from-visibility even if env is set.")
    parser.add_argument("--ks-transition-deg", type=float,
                        default=_env_float("FOV_OPT_KS_TRANSITION_DEG", 0.0),
                        help="Transition width (deg) for ks-from-visibility.")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip PNG output even if matplotlib is available.")
    parser.add_argument("--max-print", type=int, default=5,
                        help="How many top/bottom pose deltas to print.")
    parser.add_argument("--full", action="store_true",
                        help="Emit per-pose CSV with debug/jacobian columns if available.")
    parser.add_argument("--debug-log", default="",
                        help="Optional path to optimization_debug.csv to merge.")
    return parser.parse_args()


def resolve_default_points() -> str:
    env_override = os.environ.get("FOV_POINTS3D", "").strip()
    if env_override:
        return env_override
    root = Path(__file__).resolve().parents[2]
    candidates = [
        root / "act_map_exp" / "exp_data" / "warehouse_base_model_r2_a20" / "sparse" / "0" / "points3D.txt",
        root / "act_map_exp" / "exp_data" / "warehouse_base_model_r1_a30" / "sparse" / "0" / "points3D.txt",
        root / "act_map_exp" / "localization" / "warehouse_base" / "sparse" / "0" / "points3D.txt",
    ]
    for cand in candidates:
        if cand.exists():
            return str(cand)
    return ""


def main() -> None:
    args = parse_args()
    paths: List[Path] = []
    for p in args.metrics:
        paths.append(Path(p).expanduser().resolve())
    if args.root:
        root = Path(args.root).expanduser().resolve()
        if root.exists():
            paths.extend(sorted(root.rglob(args.pattern)))

    seen = set()
    unique_paths: List[Path] = []
    for p in paths:
        if p in seen:
            continue
        seen.add(p)
        unique_paths.append(p)

    if not args.include_metrics:
        unique_paths = [p for p in unique_paths if not p.name.endswith("_metrics.txt")]

    if not unique_paths:
        raise SystemExit("No quiver files found. Pass --metrics or --root.")

    points = None
    points_path = args.points or resolve_default_points()
    if not points_path:
        raise SystemExit("--points is required (no default points3D.txt found)")
    points = load_points(
        Path(points_path).expanduser().resolve(),
        parse_cols(args.points_cols),
        stride=max(1, args.point_stride),
        max_points=max(0, args.max_points),
    )
    if points.size == 0:
        raise SystemExit("No points loaded for alignment evaluation.")

    tag = "_align"

    schedule_deg = parse_schedule(args.fov_schedule)
    vis30_deg = 15.0

    def load_quiver_metrics(quiver_path: Path):
        data = parse_blocks(quiver_path, min_cols=6)
        pos = data[:, :, :3]
        dirs = data[:, :, 3:6]
        vis_used_count = None
        vis_used_score = None
        metrics_path = resolve_metrics_path(quiver_path)
        if metrics_path is not None:
            metrics_data = parse_blocks(metrics_path, min_cols=8)
            if metrics_data.shape[0] == pos.shape[0] and metrics_data.shape[1] == pos.shape[1]:
                vis_used_count = metrics_data[:, :, 6]
                vis_used_score = metrics_data[:, :, 7]
            else:
                print(
                    f"Warning: metrics file shape mismatch for {metrics_path} "
                    f"(metrics {metrics_data.shape} vs quivers {data.shape}); ignoring metrics."
                )
        alpha_deg = build_alpha_schedule(pos.shape[0], schedule_deg, args.vis_angle)
        ks_iter = build_ks_schedule(alpha_deg, args.vis_ks,
                                    args.ks_from_visibility, args.ks_transition_deg)
        align_mean, align_pos_mean, vis_sched_count, vis_sched_score, vis30_count, vis30_score = (
            compute_metrics(pos, dirs, points, alpha_deg, ks_iter, vis30_deg, args.vis_ks)
        )
        if vis_used_count is not None and vis_used_score is not None:
            vis_sched_count = vis_used_count
            vis_sched_score = vis_used_score
        align_norm_mean = (align_mean + 1.0) * 0.5
        align_mean_delta = align_mean - align_mean[0:1]
        align_pos_delta = align_pos_mean - align_pos_mean[0:1]
        align_norm_rel = normalized_progress(align_norm_mean, align_norm_mean[0:1], upper=1.0)
        align_pos_rel = normalized_progress(align_pos_mean, align_pos_mean[0:1], upper=1.0)
        return (
            alpha_deg, ks_iter,
            align_mean, align_pos_mean, align_norm_mean,
            align_mean_delta, align_pos_delta, align_norm_rel, align_pos_rel,
            vis_sched_count, vis_sched_score, vis30_count, vis30_score,
            metrics_path,
        )

    if args.variation:
        filtered = []
        for p in unique_paths:
            if args.root:
                root = Path(args.root).expanduser().resolve()
                try:
                    rel = p.relative_to(root)
                    if args.variation in rel.parts:
                        filtered.append(p)
                except ValueError:
                    if f"/{args.variation}/" in str(p):
                        filtered.append(p)
            else:
                if f"/{args.variation}/" in str(p):
                    filtered.append(p)
        unique_paths = filtered
        if not unique_paths:
            raise SystemExit(f"No quiver files found for variation: {args.variation}")
        for metrics_path in unique_paths:
            if not metrics_path.exists():
                print(f"Missing: {metrics_path}")
                continue
            (alpha_deg, ks_iter,
             align, align_pos, align_norm,
             align_delta, align_pos_delta, align_norm_rel, align_pos_rel,
             vis_sched_count, vis_sched_score, vis30_count, vis30_score,
             metrics_path_used) = load_quiver_metrics(metrics_path)
            debug_path = resolve_debug_log_path(metrics_path if metrics_path_used is None else metrics_path_used,
                                                args.debug_log)
            debug_fields, debug_arrays = load_debug_arrays(
                debug_path, align.shape[0], align.shape[1]
            )
            prefix = metrics_path.stem
            if prefix.endswith("_metrics"):
                prefix = prefix[: -len("_metrics")]
            prefix = f"{prefix}{tag}"
            out_dir = metrics_path.parent / "quiver_analysis"
            out_dir.mkdir(parents=True, exist_ok=True)
            progress_csv = out_dir / f"{prefix}_pose_metrics_progress.csv"
            progress_full_csv = out_dir / f"{prefix}_pose_metrics_progress_full.csv"
            iter_csv = out_dir / f"{prefix}_pose_metrics_per_iter_summary.csv"
            pose_csv = out_dir / f"{prefix}_pose_metrics_per_pose_summary.csv"
            plot_png = out_dir / f"{prefix}_metrics_progress.png"

            write_progress_csv(
                progress_csv, alpha_deg,
                align, align_pos, align_norm,
                align_delta, align_pos_delta, align_norm_rel,
                vis_sched_count, vis_sched_score, vis30_count, vis30_score
            )
            if args.full:
                write_progress_csv_full(
                    progress_full_csv, alpha_deg,
                    align, align_pos, align_norm,
                    align_delta, align_pos_delta, align_norm_rel,
                    vis_sched_count, vis_sched_score, vis30_count, vis30_score,
                    debug_fields, debug_arrays
                )
            write_iter_summary(
                iter_csv, alpha_deg, ks_iter,
                align, align_pos, align_norm, align_norm_rel, align_pos_rel,
                vis_sched_count, vis_sched_score, vis30_count, vis30_score
            )
            write_pose_summary(
                pose_csv,
                align, align_pos, align_norm,
                align_norm_rel, align_pos_rel,
                vis_sched_count, vis_sched_score,
                vis30_count, vis30_score
            )
            plotted = False if args.no_plots else save_plot(
                plot_png,
                align, align_pos, align_norm_rel,
                vis_sched_count, vis_sched_score,
                vis30_count, vis30_score
            )

            delta = align[-1] - align[0]
            best_idx = np.argsort(-delta)[:max(0, args.max_print)]
            worst_idx = np.argsort(delta)[:max(0, args.max_print)]
            print(f"[{metrics_path}] iterations={align.shape[0]} poses={align.shape[1]}")
            print(f"  align: first mean={align[0].mean():.3f} last mean={align[-1].mean():.3f}")
            print(f"  align+: first mean={align_pos[0].mean():.3f} last mean={align_pos[-1].mean():.3f}")
            print(f"  saved: {progress_csv.name}, {iter_csv.name}, {pose_csv.name}")
            if args.full:
                print(f"  full: {progress_full_csv.name}")
                if debug_fields:
                    print(f"  debug: {debug_path}")
                else:
                    print("  debug: not found (no jacobian columns)")
            if plotted:
                print(f"  plot: {plot_png.name}")
            else:
                print("  plot: skipped (matplotlib not available or --no-plots)")
            if args.max_print > 0:
                print("  top pose deltas (align):")
                for idx in best_idx:
                    print(f"    pose {idx}: {align[0, idx]:.3f} -> {align[-1, idx]:.3f} (d={delta[idx]:+.3f})")
                print("  bottom pose deltas (align):")
                for idx in worst_idx:
                    print(f"    pose {idx}: {align[0, idx]:.3f} -> {align[-1, idx]:.3f} (d={delta[idx]:+.3f})")
    else:
        runs = []
        alpha_first = None
        ks_first = None
        for metrics_path in unique_paths:
            if not metrics_path.exists():
                print(f"Missing: {metrics_path}")
                continue
            (alpha_deg, ks_iter,
             align, align_pos, align_norm,
             _align_delta, _align_pos_delta, align_norm_rel, align_pos_rel,
             vis_sched_count, vis_sched_score, vis30_count, vis30_score,
             _metrics_path_used) = load_quiver_metrics(metrics_path)
            if alpha_first is None:
                alpha_first = alpha_deg
                ks_first = ks_iter
            runs.append((
                align, align_pos, align_norm, align_norm_rel, align_pos_rel,
                vis_sched_count, vis_sched_score, vis30_count, vis30_score
            ))
        if not runs:
            raise SystemExit("No valid metrics files to summarize.")
        if args.root:
            out_dir = Path(args.root).expanduser().resolve()
        else:
            import os
            out_dir = Path(os.path.commonpath([str(p) for p in unique_paths]))
        out_dir = out_dir / "quiver_analysis"
        out_dir.mkdir(parents=True, exist_ok=True)
        if alpha_first is None:
            alpha_first = np.zeros((0,), dtype=float)
        if ks_first is None:
            ks_first = np.zeros((0,), dtype=float)
        summarize_combined(runs, alpha_first, ks_first, out_dir, args.no_plots, tag)


if __name__ == "__main__":
    main()
