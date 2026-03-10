#!/usr/bin/env python3
import argparse
import json
import math
import shutil
import subprocess
import time
from pathlib import Path
import os 
import itertools
import random


def load_yaml(path: Path):
    import yaml  # type: ignore
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return {}
    return data


def normalize_cfg(cfg):
    if not isinstance(cfg, dict):
        return {}
    out = dict(cfg)

    def alias_key(src, dst):
        if src in out and dst not in out:
            out[dst] = out[src]
        out.pop(src, None)

    alias_key("tune_max_iter", "tune_max_iteration")
    alias_key("max_iter_range", "max_iteration_range")
    alias_key("tune_traj_jac_step", "tune_trajectory_jacobian_step")
    alias_key("traj_jac_step_range", "trajectory_jacobian_step_range")

    for key in ("fixed_params", "initial_params"):
        raw = out.get(key)
        if not isinstance(raw, dict):
            continue
        params = dict(raw)
        if "max_iter" in params and "max_iteration" not in params:
            params["max_iteration"] = params["max_iter"]
        if "traj_jac_step" in params and "trajectory_jacobian_step" not in params:
            params["trajectory_jacobian_step"] = params["traj_jac_step"]
        params.pop("max_iter", None)
        params.pop("traj_jac_step", None)
        out[key] = params

    return out


def suggest_ks(alpha_deg: float, transition_deg: float):
    if alpha_deg <= 0.0 or transition_deg <= 0.0:
        return None
    alpha_rad = math.radians(alpha_deg)
    delta_rad = math.radians(transition_deg)
    sin_alpha = math.sin(alpha_rad)
    if abs(sin_alpha) < 1e-6 or delta_rad <= 0.0:
        return None
    return 2.94 / (delta_rad * sin_alpha)


def parse_schedule(schedule):
    if schedule is None:
        return []
    if isinstance(schedule, (list, tuple)):
        return [float(v) for v in schedule]
    if isinstance(schedule, str):
        out = []
        for part in schedule.split(","):
            part = part.strip()
            if not part:
                continue
            out.append(float(part))
        return out
    return []


def build_param_sets(cfg):
    base = dict(cfg.get("initial_params") or {})
    fov_opts = cfg.get("fov_schedule_options") or []
    step_opts = cfg.get("step_norm_mode_options") or []
    if "fov_schedule" not in base and fov_opts:
        base["fov_schedule"] = fov_opts[0]
    if "step_norm_mode" not in base and step_opts:
        base["step_norm_mode"] = step_opts[0]
    params = [base]
    for sched in fov_opts:
        p = dict(base)
        p["fov_schedule"] = sched
        params.append(p)
    for mode in step_opts:
        p = dict(base)
        p["step_norm_mode"] = mode
        params.append(p)

    seen = set()
    unique = []
    for p in params:
        key = tuple(sorted((k, json.dumps(v, sort_keys=True)) for k, v in p.items()))
        if key in seen:
            continue
        seen.add(key)
        unique.append(p)
    return unique


def _specs_from_cfg(cfg):
    specs = []
    fixed = cfg.get("fixed_params") or {}

    def add_cat(name, options):
        if not options:
            return
        if name in fixed:
            return
        specs.append(("cat", name, options))

    def add_range(name, rng, log=False, is_int=False):
        if not rng or len(rng) < 2:
            return
        if name in fixed:
            return
        specs.append(("range", name, (rng[0], rng[1], log, is_int)))

    if cfg.get("tune_fov_schedule"):
        add_cat("fov_schedule", cfg.get("fov_schedule_options") or [])
    if cfg.get("tune_step_norm_mode"):
        add_cat("step_norm_mode", cfg.get("step_norm_mode_options") or [])

    if cfg.get("tune_base_step"):
        add_range("base_step_scale", cfg.get("base_step_scale_range"), log=cfg.get("base_step_scale_log", False))
    if cfg.get("tune_min_step_deg"):
        add_range("min_step_deg", cfg.get("min_step_deg_range"))
    if cfg.get("tune_max_step_deg"):
        add_range("max_step_deg", cfg.get("max_step_deg_range"))
    if cfg.get("tune_trajectory_jacobian_step"):
        add_range("trajectory_jacobian_step", cfg.get("trajectory_jacobian_step_range"))
    if cfg.get("tune_max_iteration"):
        add_range("max_iteration", cfg.get("max_iteration_range"), is_int=True)
    if cfg.get("tune_ks"):
        add_range("ks", cfg.get("ks_range"), log=cfg.get("ks_log", False))
    if cfg.get("tune_ks_transition_deg"):
        add_range("ks_transition_deg", cfg.get("ks_transition_deg_range"))

    return specs


def _apply_ks_from_visibility(cfg, params):
    if not cfg.get("ks_from_visibility"):
        return
    if "ks" in params:
        return
    sched = parse_schedule(params.get("fov_schedule"))
    if not sched:
        return
    alpha = sched[-1]
    transition = params.get("ks_transition_deg", cfg.get("ks_transition_deg", 0))
    ks = suggest_ks(float(alpha), float(transition))
    if ks is not None:
        params["ks"] = ks


def build_param_sets_grid(cfg, n_steps):
    base = dict(cfg.get("initial_params") or {})
    fixed = cfg.get("fixed_params") or {}
    base.update(fixed)
    specs = _specs_from_cfg(cfg)
    axes = []
    for spec in specs:
        kind, name, info = spec
        if kind == "cat":
            axes.append([(name, v) for v in info])
        else:
            vmin, vmax, log, is_int = info
            if n_steps <= 1:
                vals = [vmin]
            else:
                if log and vmin > 0 and vmax > 0:
                    vals = [vmin * ((vmax / vmin) ** (i / (n_steps - 1))) for i in range(n_steps)]
                else:
                    vals = [vmin + (vmax - vmin) * (i / (n_steps - 1)) for i in range(n_steps)]
            if is_int:
                vals = sorted({int(round(v)) for v in vals})
            axes.append([(name, v) for v in vals])

    if not axes:
        params = dict(base)
        _apply_ks_from_visibility(cfg, params)
        return [params]

    out = []
    for combo in itertools.product(*axes):
        params = dict(base)
        for name, val in combo:
            params[name] = val
        _apply_ks_from_visibility(cfg, params)
        out.append(params)
    return out


def build_param_sets_random(cfg, n_samples, seed):
    base = dict(cfg.get("initial_params") or {})
    fixed = cfg.get("fixed_params") or {}
    base.update(fixed)
    specs = _specs_from_cfg(cfg)
    rng = random.Random(seed)
    out = []
    seen = set()
    attempts = 0
    while len(out) < n_samples and attempts < n_samples * 50:
        attempts += 1
        params = dict(base)
        for spec in specs:
            kind, name, info = spec
            if kind == "cat":
                params[name] = rng.choice(info)
            else:
                vmin, vmax, log, is_int = info
                if log and vmin > 0 and vmax > 0:
                    r = rng.random()
                    val = vmin * ((vmax / vmin) ** r)
                else:
                    val = vmin + (vmax - vmin) * rng.random()
                if is_int:
                    val = int(round(val))
                params[name] = val
        _apply_ks_from_visibility(cfg, params)
        key = tuple(sorted((k, json.dumps(v, sort_keys=True)) for k, v in params.items()))
        if key in seen:
            continue
        seen.add(key)
        out.append(params)
    return out


def param_label(idx, params):
    parts = [f"{idx:02d}"]
    sched = params.get("fov_schedule")
    if sched is not None:
        sched_str = sched if isinstance(sched, str) else ",".join(str(v) for v in sched)
        parts.append("fov" + sched_str.replace(",", "-"))
    mode = params.get("step_norm_mode")
    if mode:
        parts.append(f"norm{mode}")
    bss = params.get("base_step_scale")
    if bss is not None:
        parts.append(f"bss{bss:.2f}")
    ms = params.get("min_step_deg")
    mx = params.get("max_step_deg")
    if ms is not None and mx is not None:
        parts.append(f"step{ms:.2f}-{mx:.2f}")
    tj = params.get("trajectory_jacobian_step")
    if tj is not None:
        parts.append(f"jac{tj:.2f}")
    label = "_".join(parts)
    return label.replace(".", "p")


def ensure_empty_dir(path: Path, force: bool):
    if path.exists():
        if not force and any(path.iterdir()):
            raise SystemExit(f"Output dir not empty: {path} (use --force to reuse)")
    path.mkdir(parents=True, exist_ok=True)


def copy_inputs(src_root: Path, dst_root: Path, views):
    for view in views:
        src = src_root / view / f"{view}_none" / "stamped_Twc.txt"
        if not src.exists():
            raise SystemExit(f"Missing input stamped_Twc.txt: {src}")
        dst_dir = dst_root / view / f"{view}_none"
        dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst_dir / "stamped_Twc.txt")


def write_analysis_cfg(debug_dir: Path):
    cfg_path = debug_dir / "analysis_cfg_debug.yaml"
    config = {
        "colors": {
            "along_path": "red",
            "optimized_path_yaw": "blue",
        },
        "linestyles": {
            "along_path": "dotted",
            "optimized_path_yaw": "solid",
        },
        "labels": {
            "along_path": "Input (along)",
            "optimized_path_yaw": "Optimized (along)",
        },
        "ordered_types": ["along_path", "optimized_path_yaw"],
        "hist_max_trans_e": 1.0,
        "hist_max_rot_e": 10.0,
        "max_trans_e_m": 0.5,
        "max_rot_e_deg": 10.0,
    }
    import yaml  # type: ignore
    cfg_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    return cfg_path


def parse_views(raw: str):
    if not raw:
        return []
    raw = raw.replace(" ", ",")
    parts = [v.strip() for v in raw.split(",") if v.strip()]
    return parts


def _read_data_lines(path: Path):
    lines = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            lines.append(line)
    return lines


def _select_existing(path_dir: Path, candidates):
    for name in candidates:
        cand = path_dir / name
        if cand.exists():
            return cand
    return None


def _write_pose_errors(path: Path, lines):
    with path.open("w", encoding="utf-8") as f:
        f.write("# image_name trans_e_m rot_e_deg\n")
        for line in lines:
            f.write(line + "\n")


def _write_twc(path: Path, lines):
    with path.open("w", encoding="utf-8") as f:
        f.write("# transformation matrices - {} entries: time; mat of size: 4 x 4\n".format(
            len(lines)
        ))
        for line in lines:
            f.write(line + "\n")


def aggregate_variant(run_dir: Path, views, src_name: str, dst_name: str,
                      twc_candidates, err_candidates):
    twc_lines = []
    err_lines = []
    for view in views:
        src_dir = run_dir / view / f"{view}_none" / src_name
        if not src_dir.is_dir():
            raise SystemExit(f"Missing source dir: {src_dir}")
        err_file = _select_existing(src_dir, err_candidates)
        if err_file is None:
            raise SystemExit(f"Missing pose errors in {src_dir} (checked {err_candidates})")
        twc_file = _select_existing(src_dir, twc_candidates)
        if twc_file is None:
            raise SystemExit(f"Missing Twc in {src_dir} (checked {twc_candidates})")
        err_i = _read_data_lines(err_file)
        twc_i = _read_data_lines(twc_file)
        if len(err_i) != len(twc_i):
            raise SystemExit(
                f"Mismatch lines in {src_dir}: pose_errors={len(err_i)} twc={len(twc_i)}"
            )
        err_lines.extend(err_i)
        twc_lines.extend(twc_i)

    dst_dir = run_dir / dst_name
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    dst_dir.mkdir(parents=True, exist_ok=True)
    _write_pose_errors(dst_dir / "pose_errors.txt", err_lines)
    _write_twc(dst_dir / "stamped_Twc_path_yaw.txt", twc_lines)
    return dst_dir


def _limit_registration_lists(rel_image_list: Path, rel_cam_list: Path, max_reg_images: int):
    if max_reg_images is None or max_reg_images <= 0:
        return None
    if not rel_image_list.exists() or not rel_cam_list.exists():
        return None

    img_lines = [ln for ln in rel_image_list.read_text(encoding="utf-8").splitlines() if ln.strip()]
    cam_lines = [ln for ln in rel_cam_list.read_text(encoding="utf-8").splitlines() if ln.strip()]
    n = min(len(img_lines), len(cam_lines))
    if n == 0:
        return (0, 0)
    if n <= max_reg_images:
        return (n, n)

    if max_reg_images == 1:
        idx = [0]
    else:
        idx = [(i * (n - 1)) // (max_reg_images - 1) for i in range(max_reg_images)]

    img_sel = [img_lines[i] for i in idx]
    cam_sel = [cam_lines[i] for i in idx]

    rel_image_list.write_text("\n".join(img_sel) + "\n", encoding="utf-8")
    rel_cam_list.write_text("\n".join(cam_sel) + "\n", encoding="utf-8")
    return (n, len(img_sel))


def _ensure_base_model_copy(src: Path, dst: Path):
    if dst.exists():
        return dst
    shutil.copytree(src, dst)
    return dst


def _find_ue_pose_file(input_dir: Path, path_suffix: str):
    candidates = [
        input_dir / f"stamped_Twc_ue{path_suffix}.txt",
        input_dir / f"optimized_stamped_Twc_ue{path_suffix}.txt",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    checked = ", ".join(str(c) for c in candidates)
    raise SystemExit(f"Cannot find UE poses (checked: {checked})")


def _run_variant_pipeline(variant_dir: Path, reg_name: str, base_model: Path,
                          colmap_dir: Path, max_reg_images: int,
                          acc_trans: float = 0.5, acc_rot: float = 10.0):
    render_sc = colmap_dir / "my_render_ue.py"
    gen_list_sc = colmap_dir / "generate_img_rel_path.py"
    reg_sc = colmap_dir / "register_images_to_model.py"
    cal_e_sc = colmap_dir / "calculate_pose_errors.py"

    ue_pose = _find_ue_pose_file(variant_dir, "_path_yaw")
    render_dir = variant_dir / "rendering"
    if render_dir.exists():
        shutil.rmtree(render_dir)

    cmd_render = ["python3", str(render_sc), str(ue_pose), "--save_dir", str(render_dir)]
    subprocess.run(cmd_render, check=True)

    img_dir = render_dir / "images"
    cam_list = render_dir / "img_nm_to_colmap_cam.txt"
    if not cam_list.exists():
        raise SystemExit(f"Missing camera list: {cam_list}")

    cmd_gen = [
        "python3",
        str(gen_list_sc),
        "--base_dir",
        str(base_model / "images"),
        "--img_dir",
        str(img_dir),
        "--img_nm_to_cam_list",
        str(cam_list),
    ]
    subprocess.run(cmd_gen, check=True)
    rel_img_list = img_dir / "rel_img_path.txt"
    rel_cam_list = img_dir / "rel_img_nm_to_cam_list.txt"
    if max_reg_images and max_reg_images > 0:
        _limit_registration_lists(rel_img_list, rel_cam_list, max_reg_images)

    cmd_reg = [
        "python3",
        str(reg_sc),
        str(base_model),
        "--reg_name",
        reg_name,
        "--reg_list_fn",
        str(rel_img_list),
        "--img_nm_to_colmap_cam_list",
        str(rel_cam_list),
        "--upref_no_time",
        "--min_num_inliers",
        "10",
    ]
    subprocess.run(cmd_reg, check=True)

    reg_model_dir = base_model / f"{reg_name}_sparse"
    img_name_to_colmap = render_dir / "img_name_to_colmap_Tcw.txt"
    if not img_name_to_colmap.exists():
        raise SystemExit(f"Missing pose file: {img_name_to_colmap}")

    cmd_eval = [
        "python3",
        str(cal_e_sc),
        "--reg_model_dir",
        str(reg_model_dir),
        "--reg_img_name_to_colmap_Tcw",
        str(img_name_to_colmap),
        "--reg_img_dir",
        str(img_dir),
        "--output_path",
        str(variant_dir),
        "--acc_trans_thresh",
        str(acc_trans),
        "--acc_rot_thresh",
        str(acc_rot),
    ]
    subprocess.run(cmd_eval, check=True)


def _load_pose_errors(path: Path, max_trans=None, max_rot=None):
    trans = []
    rot = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                te = float(parts[1])
                re = float(parts[2])
            except ValueError:
                continue
            if max_trans is not None and math.isfinite(max_trans) and te > max_trans:
                te = float('nan')
            if max_rot is not None and math.isfinite(max_rot) and re > max_rot:
                re = float('nan')
            trans.append(te)
            rot.append(re)
    total = len(trans)
    nan_count = sum(1 for v in trans if not math.isfinite(v))
    nan_ratio = float('nan') if total == 0 else nan_count / total
    return trans, rot, nan_ratio, total


def _finite_stats(values):
    if not values:
        return float('nan'), float('nan'), 0
    import numpy as np  # local import
    arr = np.asarray(values, dtype=float)
    return float(np.mean(arr)), float(np.std(arr)), int(arr.size)


def _replace_nan(values, fill_val):
    return [fill_val if not math.isfinite(v) else v for v in values]


def _plot_comparison(out_dir: Path, labels, trans_sets, rot_sets, nan_ratios, totals,
                     hist_max_trans: float, hist_max_rot: float,
                     penalty_trans: float, penalty_rot: float):
    os.environ.setdefault("MPLCONFIGDIR", str(out_dir / ".mpl"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    out_dir.mkdir(parents=True, exist_ok=True)

    # Colors: along_path in black, others from tab10.
    colors = ['#000000']
    cmap = plt.get_cmap('tab10')
    for i in range(max(0, len(labels) - 1)):
        colors.append(cmap(i % 10))

    plot_max_trans = hist_max_trans * 1.2
    plot_max_rot = hist_max_rot * 1.2

    modes = {
        "finite": ("finite", None, None),
        "original": ("original", plot_max_trans, plot_max_rot),
        "penalized": ("penalized", penalty_trans, penalty_rot),
    }

    x = np.arange(len(labels))

    def _annotate(ax, bars, fmt="{:.2f}", as_percent=False, fontsize=8):
        for rect in bars:
            height = rect.get_height()
            if not math.isfinite(height):
                continue
            label = "{:.1f}%".format(height * 100.0) if as_percent else fmt.format(height)
            ax.annotate(
                label,
                xy=(rect.get_x() + rect.get_width() / 2.0, height),
                xytext=(0, 2),
                textcoords="offset points",
                ha="center",
                va="bottom",
                fontsize=fontsize,
                clip_on=False,
            )
    for mode, (mode_name, trans_fill, rot_fill) in modes.items():
        # Build mode-specific arrays.
        if mode == "finite":
            trans_plot = [[v for v in arr if math.isfinite(v)] for arr in trans_sets]
            rot_plot = [[v for v in arr if math.isfinite(v)] for arr in rot_sets]
        else:
            trans_plot = [_replace_nan(v, trans_fill) for v in trans_sets]
            rot_plot = [_replace_nan(v, rot_fill) for v in rot_sets]

        # Overall hist (CDF).
        fig = plt.figure(figsize=(12, 6))
        axes = fig.subplots(1, 2, sharey=True)
        pos_ax, rot_ax = axes[0], axes[1]
        for i, label in enumerate(labels):
            pos_ax.hist(trans_plot[i], label=label, bins=50, density=True, histtype='step',
                        cumulative=True, range=(0, plot_max_trans), color=colors[i])
            rot_ax.hist(rot_plot[i], label=label, bins=50, density=True, histtype='step',
                        cumulative=True, range=(0, plot_max_rot), color=colors[i])
        pos_ax.set_xlabel('Position error (m)')
        rot_ax.set_xlabel('Rotation error (deg)')
        pos_ax.set_xlim([0, hist_max_trans])
        rot_ax.set_xlim([0, hist_max_rot])
        pos_ax.set_ylim([0, 1.01])
        rot_ax.set_ylim([0, 1.01])
        pos_ax.set_yticks(np.linspace(0, 1.0, 5))
        pos_ax.set_yticklabels([f"{int(v*100)}%" for v in np.linspace(0, 1.0, 5)])
        fig.legend(loc='lower right', ncol=2, fontsize='small')
        fig.tight_layout()
        fig.savefig(out_dir / f"overall_hist_{mode_name}.png", bbox_inches='tight', pad_inches=0.25, dpi=300)
        plt.close(fig)

        # Mean/std plot.
        means_t = []
        stds_t = []
        counts_t = []
        means_r = []
        stds_r = []
        counts_r = []
        for t_vals, r_vals in zip(trans_plot, rot_plot):
            mt, st, ct = _finite_stats(t_vals) if mode == "finite" else _finite_stats(t_vals)
            mr, sr, cr = _finite_stats(r_vals) if mode == "finite" else _finite_stats(r_vals)
            means_t.append(0.0 if not math.isfinite(mt) else mt)
            stds_t.append(0.0 if not math.isfinite(st) else st)
            counts_t.append(ct)
            means_r.append(0.0 if not math.isfinite(mr) else mr)
            stds_r.append(0.0 if not math.isfinite(sr) else sr)
            counts_r.append(cr)

        fig_ms = plt.figure(figsize=(12, 6))
        axes_ms = fig_ms.subplots(2, 1, sharex=True)
        bars_t = axes_ms[0].bar(x, means_t, yerr=stds_t, color='#4c72b0', capsize=3)
        axes_ms[0].set_ylabel('pos mean ± std')
        bars_r = axes_ms[1].bar(x, means_r, yerr=stds_r, color='#dd8452', capsize=3)
        axes_ms[1].set_ylabel('rot mean ± std')
        axes_ms[1].set_xlabel('variation')
        axes_ms[1].set_xticks(x)
        axes_ms[1].set_xticklabels(labels, rotation=45, ha='right')
        _annotate(axes_ms[0], bars_t, fmt="{:.2f}")
        _annotate(axes_ms[1], bars_r, fmt="{:.2f}")
        fig_ms.tight_layout()
        fig_ms.savefig(out_dir / f"overall_mean_std_{mode_name}.png", bbox_inches='tight', pad_inches=0.25, dpi=300)
        plt.close(fig_ms)

        # Summary CSV.
        summary_fn = out_dir / f"overall_summary_{mode_name}.csv"
        with summary_fn.open("w", encoding="utf-8") as f:
            f.write("label,trans_mean,trans_std,trans_count,rot_mean,rot_std,rot_count,nan_ratio,total\n")
            for i, label in enumerate(labels):
                f.write("{},{:.6g},{:.6g},{},{:.6g},{:.6g},{},{:.6g},{}\n".format(
                    label,
                    means_t[i] if math.isfinite(means_t[i]) else float('nan'),
                    stds_t[i] if math.isfinite(stds_t[i]) else float('nan'),
                    counts_t[i],
                    means_r[i] if math.isfinite(means_r[i]) else float('nan'),
                    stds_r[i] if math.isfinite(stds_r[i]) else float('nan'),
                    counts_r[i],
                    nan_ratios[i] if math.isfinite(nan_ratios[i]) else float('nan'),
                    totals[i],
                ))

    # Nan ratio plot.
    fig_nan = plt.figure(figsize=(10, 4))
    ax_nan = fig_nan.add_subplot(111)
    nan_plot = [0.0 if not math.isfinite(v) else v for v in nan_ratios]
    bars_nan = ax_nan.bar(x, nan_plot, color='#7f7f7f')
    ax_nan.set_ylabel('nan ratio')
    ax_nan.set_ylim(0, 1.05)
    ax_nan.set_xticks(x)
    ax_nan.set_xticklabels(labels, rotation=45, ha='right')
    _annotate(ax_nan, bars_nan, as_percent=True, fontsize=8)
    fig_nan.tight_layout()
    fig_nan.savefig(out_dir / "overall_nan_ratio.png", bbox_inches='tight', pad_inches=0.25, dpi=300)
    plt.close(fig_nan)

    # Nan ratio plot (same for all modes).
    fig_nan = plt.figure(figsize=(10, 4))
    ax_nan = fig_nan.add_subplot(111)
    nan_plot = [0.0 if not math.isfinite(v) else v for v in nan_ratios]
    ax_nan.bar(x, nan_plot, color='#7f7f7f')
    ax_nan.set_ylabel('nan ratio')
    ax_nan.set_ylim(0, 1.05)
    ax_nan.set_xticks(x)
    ax_nan.set_xticklabels(labels, rotation=45, ha='right')
    fig_nan.tight_layout()
    fig_nan.savefig(out_dir / "overall_nan_ratio.png", bbox_inches='tight', pad_inches=0.25, dpi=300)
    plt.close(fig_nan)


def _copy_cache_outputs(src_dir: Path, dst_dir: Path):
    for name in ("pose_errors.txt", "registered_poses_ue.txt", "pose_metrics.json"):
        src = src_dir / name
        if src.exists():
            shutil.copy2(src, dst_dir / name)


def main():
    parser = argparse.ArgumentParser(
        description="Debug pipeline: optimize along-path for selected views with multiple param sets."
    )
    parser.add_argument("--dataset", choices=["r1_a30", "r2_a20"], default="r2_a20")
    parser.add_argument("--outdir", default="", help="Output debug directory.")
    parser.add_argument("--views", default="diagonal",
                        help="Comma-separated list of views (e.g. top,diagonal)")
    parser.add_argument("--grid", action="store_true",
                        help="Use grid sampling from ranges/options in optuna_fov_tune.yaml.")
    parser.add_argument("--random", action="store_true",
                        help="Use random sampling from ranges/options in optuna_fov_tune.yaml.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--n", type=int, default=0,
                        help="Number of samples (random) or steps per range (grid).")
    parser.add_argument("--max-sets", type=int, default=0, help="Limit number of param sets.")
    parser.add_argument("--max-reg-images", type=int, default=0)
    parser.add_argument("--keep-raw", action="store_true",
                        help="Keep raw per-view folders instead of moving to .raw_views.")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    root_dir = Path(__file__).resolve().parents[2]
    debug_dir = Path(args.outdir) if args.outdir else (root_dir / "fov_opt" / "debug" / time.strftime("%Y%m%d_%H%M%S"))
    ensure_empty_dir(debug_dir, args.force)

    opt_cfg = normalize_cfg(load_yaml(root_dir / "fov_opt" / "optuna_fov_tune.yaml"))
    if args.grid and args.random:
        raise SystemExit("Use only one of --grid or --random.")
    if args.grid:
        steps = args.n if args.n and args.n > 0 else 3
        param_sets = build_param_sets_grid(opt_cfg, steps)
    elif args.random:
        samples = args.n if args.n and args.n > 0 else 10
        param_sets = build_param_sets_random(opt_cfg, samples, args.seed)
    else:
        param_sets = build_param_sets(opt_cfg)
    if args.max_sets and args.max_sets > 0:
        param_sets = param_sets[: args.max_sets]

    views = parse_views(args.views)
    if not views:
        raise SystemExit("No views provided; use --views top,diagonal (comma-separated).")
    ana_cfg = write_analysis_cfg(debug_dir)
    ana_cfg_data = load_yaml(ana_cfg)
    hist_max_trans = float(ana_cfg_data.get("hist_max_trans_e", 1.0))
    hist_max_rot = float(ana_cfg_data.get("hist_max_rot_e", 10.0))
    acc_trans = float(ana_cfg_data.get("max_trans_e_m", 0.5))
    acc_rot = float(ana_cfg_data.get("max_rot_e_deg", 10.0))

    if args.dataset == "r1_a30":
        trace_src = root_dir / "act_map_exp" / "trace_r1_a30"
        points3d = root_dir / "act_map_exp" / "exp_data" / "warehouse_base_model_r1_a30" / "sparse" / "0" / "points3D.txt"
        base_model = root_dir / "act_map_exp" / "exp_data" / "warehouse_base_model_r1_a30"
    else:
        trace_src = root_dir / "act_map_exp" / "trace_r2_a20"
        points3d = root_dir / "act_map_exp" / "exp_data" / "warehouse_base_model_r2_a20" / "sparse" / "0" / "points3D.txt"
        base_model = root_dir / "act_map_exp" / "exp_data" / "warehouse_base_model_r2_a20"

    colmap_dir = root_dir / "act_map_exp" / "colmap_scripts"
    run_opt = root_dir / "act_map_exp" / "scripts" / "run_fov_opt_rrt_none.sh"
    base_model = _ensure_base_model_copy(base_model, debug_dir / f"base_model_{args.dataset}")

    summary = []
    cache_root = debug_dir / ".cache_along_path"
    cache_root.mkdir(parents=True, exist_ok=True)
    for idx, params in enumerate(param_sets, start=1):
        label = str(idx)
        run_dir = debug_dir / label
        run_dir.mkdir(parents=True, exist_ok=True)

        # Prepare minimal inputs.
        copy_inputs(trace_src, run_dir, views)

        # Resolve ks from fov schedule if requested.
        if opt_cfg.get("ks_from_visibility") and "ks" not in params:
            sched = parse_schedule(params.get("fov_schedule"))
            alpha = sched[-1] if sched else None
            transition = params.get("ks_transition_deg", opt_cfg.get("ks_transition_deg", 0))
            if alpha is not None:
                ks = suggest_ks(float(alpha), float(transition))
                if ks is not None:
                    params["ks"] = ks

        params_path = run_dir / "params.json"
        params_payload = {
            "id": idx,
            "label": param_label(idx, params),
            "params": params,
        }
        params_path.write_text(json.dumps(params_payload, indent=2), encoding="utf-8")

        env = dict(os.environ)
        env["FOV_TRACE_ROOT"] = str(run_dir)
        env["FOV_POINTS3D"] = str(points3d)
        if "max_iteration" in params:
            env["FOV_OPT_MAX_ITERATION"] = str(params["max_iteration"])
        if "ks" in params:
            env["FOV_OPT_KS"] = str(params["ks"])
        if "base_step_scale" in params:
            env["FOV_OPT_BASE_STEP_SCALE"] = str(params["base_step_scale"])
        if "min_step_deg" in params:
            env["FOV_OPT_MIN_STEP_DEG"] = str(params["min_step_deg"])
        if "max_step_deg" in params:
            env["FOV_OPT_MAX_STEP_DEG"] = str(params["max_step_deg"])
        if "step_norm_mode" in params:
            env["FOV_OPT_STEP_NORM_MODE"] = str(params["step_norm_mode"])
        if "trajectory_jacobian_step" in params:
            env["FOV_OPT_TRAJECTORY_JACOBIAN_STEP"] = str(
                params["trajectory_jacobian_step"]
            )
        if "fov_schedule" in params:
            sched = params["fov_schedule"]
            if isinstance(sched, (list, tuple)):
                sched = ",".join(str(v) for v in sched)
            env["FOV_OPT_FOV_SCHEDULE"] = str(sched)

        # Optimization (along-path only).
        cmd_opt = [str(run_opt), "--along-path"] + views
        subprocess.run(cmd_opt, check=True, cwd=root_dir / "act_map_exp" / "scripts", env=env)

        # Registration + evaluation for along_path (cached) and optimized_path_yaw.
        for view in views:
            view_root = run_dir / view / f"{view}_none"
            along_dir = view_root / "along_path"
            opt_dir = view_root / "optimized_path_yaw"
            if not along_dir.exists():
                raise SystemExit(f"Missing along_path dir: {along_dir}")
            if not opt_dir.exists():
                raise SystemExit(f"Missing optimized_path_yaw dir: {opt_dir}")
            cache_view = cache_root / view / "along_path"
            cache_view.mkdir(parents=True, exist_ok=True)
            cache_pose = cache_view / "pose_errors.txt"
            if cache_pose.exists():
                _copy_cache_outputs(cache_view, along_dir)
            else:
                _run_variant_pipeline(
                    along_dir,
                    f"{label}_{view}_along_path",
                    base_model,
                    colmap_dir,
                    args.max_reg_images,
                    acc_trans=acc_trans,
                    acc_rot=acc_rot,
                )
                _copy_cache_outputs(along_dir, cache_view)
            _run_variant_pipeline(
                opt_dir,
                f"{label}_{view}_opt_path_yaw",
                base_model,
                colmap_dir,
                args.max_reg_images,
                acc_trans=acc_trans,
                acc_rot=acc_rot,
            )

        # Aggregate along_path + optimized_path_yaw into top-level folders.
        aggregate_variant(
            run_dir,
            views,
            "along_path",
            "along_path",
            ["stamped_Twc_path_yaw.txt", "stamped_Twc.txt"],
            ["pose_errors.txt", "pose_errors_path_yaw.txt"],
        )
        aggregate_variant(
            run_dir,
            views,
            "optimized_path_yaw",
            "optimized_along_path",
            ["optimized_stamped_Twc_path_yaw.txt", "stamped_Twc_path_yaw.txt", "stamped_Twc.txt"],
            ["pose_errors.txt", "pose_errors_path_yaw.txt"],
        )

        # Hide raw view folders unless requested.
        if not args.keep_raw:
            raw_dir = run_dir / ".raw_views"
            raw_dir.mkdir(parents=True, exist_ok=True)
            for view in views:
                src = run_dir / view
                if src.exists():
                    dst = raw_dir / view
                    if dst.exists():
                        shutil.rmtree(dst)
                    shutil.move(str(src), str(dst))

        summary.append({"id": idx, "label": param_label(idx, params), "params": params})

    (debug_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    # Top-level comparison across param sets.
    analysis_dir = debug_dir / "analysis_outputs"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    labels = ["along_path"] + [str(i) for i in range(1, len(param_sets) + 1)]
    trans_sets = []
    rot_sets = []
    nan_ratios = []
    totals = []

    along_ref = None
    for idx in range(1, len(param_sets) + 1):
        run_dir = debug_dir / str(idx)
        along_dir = run_dir / "along_path" / "pose_errors.txt"
        if not along_dir.exists():
            raise SystemExit(f"Missing along_path pose_errors: {along_dir}")
        if along_ref is None:
            along_ref = along_dir.read_text(encoding="utf-8")
        else:
            if along_dir.read_text(encoding="utf-8") != along_ref:
                print(f"Warning: along_path pose_errors differ for variation {idx}.")

    # Baseline along_path from the first variation.
    base_along = debug_dir / "1" / "along_path" / "pose_errors.txt"
    t_vals, r_vals, nan_ratio, total = _load_pose_errors(
        base_along, max_trans=acc_trans, max_rot=acc_rot
    )
    trans_sets.append(t_vals)
    rot_sets.append(r_vals)
    nan_ratios.append(nan_ratio)
    totals.append(total)

    for idx in range(1, len(param_sets) + 1):
        opt_dir = debug_dir / str(idx) / "optimized_along_path" / "pose_errors.txt"
        if not opt_dir.exists():
            raise SystemExit(f"Missing optimized_along_path pose_errors: {opt_dir}")
        t_vals, r_vals, nan_ratio, total = _load_pose_errors(
            opt_dir, max_trans=acc_trans, max_rot=acc_rot
        )
        trans_sets.append(t_vals)
        rot_sets.append(r_vals)
        nan_ratios.append(nan_ratio)
        totals.append(total)

    _plot_comparison(analysis_dir, labels, trans_sets, rot_sets, nan_ratios, totals,
                     hist_max_trans, hist_max_rot, acc_trans, acc_rot)
    print(f"Done. Results under {debug_dir}")


if __name__ == "__main__":
    main()
