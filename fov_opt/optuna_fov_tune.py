#!/usr/bin/env python3
import argparse
import csv
import json
import os
import shlex
import statistics
import subprocess
import time
import math
from pathlib import Path
import sys
import optuna


STAT_INDEX = {
    "min": 0,
    "max": 1,
    "avg": 2,
    "std": 3,
}
POSE_ERROR_MODES = {"finite", "original", "penalized"}

SUPPORTED_FIXED_PARAMS = {
    "max_iteration",
    "ks",
    "ks_transition_deg",
    "base_step_scale",
    "min_step_deg",
    "max_step_deg",
    "step_norm_mode",
    "trajectory_jacobian_step",
    "fov_schedule",
}

# Legacy aliases (short names) -> canonical names.
_LEGACY_PARAM_ALIASES = {
    "max_iter": "max_iteration",
    "traj_jac_step": "trajectory_jacobian_step",
}
_LEGACY_RANGE_ALIASES = {
    "max_iter_range": "max_iteration_range",
    "traj_jac_step_range": "trajectory_jacobian_step_range",
}
_LEGACY_TUNE_ALIASES = {
    "tune_max_iter": "tune_max_iteration",
    "tune_traj_jac_step": "tune_trajectory_jacobian_step",
}


def _apply_legacy_aliases_dict(raw):
    if not isinstance(raw, dict):
        return raw
    out = dict(raw)
    for src, dst in _LEGACY_PARAM_ALIASES.items():
        if src in out and dst not in out:
            out[dst] = out[src]
        if src in out:
            out.pop(src, None)
    return out


def _apply_config_aliases(merged):
    if not isinstance(merged, dict):
        return merged
    out = dict(merged)
    for src, dst in _LEGACY_PARAM_ALIASES.items():
        if src in out and dst not in out:
            out[dst] = out[src]
        out.pop(src, None)
    for src, dst in _LEGACY_RANGE_ALIASES.items():
        if src in out and dst not in out:
            out[dst] = out[src]
        out.pop(src, None)
    for src, dst in _LEGACY_TUNE_ALIASES.items():
        if src in out and dst not in out:
            out[dst] = out[src]
        out.pop(src, None)
    for key in ("fixed_params", "initial_params"):
        if key in out:
            out[key] = _apply_legacy_aliases_dict(out[key])
    return out


def parse_error_stat(path: Path, stat: str):
    idx = STAT_INDEX[stat]
    with path.open("r", encoding="utf-8") as handle:
        lines = [line.strip() for line in handle.readlines() if line.strip()]
    if len(lines) < 3:
        raise RuntimeError(f"error_stat file looks short: {path}")
    t_vals = [float(v) for v in lines[1].split()]
    r_vals = [float(v) for v in lines[2].split()]
    if len(t_vals) < 4 or len(r_vals) < 4:
        raise RuntimeError(f"error_stat format unexpected: {path}")
    return t_vals[idx], r_vals[idx]


def parse_error_stat_files(paths, stat: str):
    te_vals = []
    re_vals = []
    for path in paths:
        t, r = parse_error_stat(Path(path), stat)
        te_vals.append(t)
        re_vals.append(r)
    if not te_vals or not re_vals:
        raise RuntimeError("error_stat files have no usable entries.")
    return statistics.mean(te_vals), statistics.mean(re_vals)


def _find_analysis_cfg(start: Path):
    cur = start.resolve()
    for parent in [cur] + list(cur.parents):
        cand = parent / "analysis_cfg.yaml"
        if cand.exists():
            return cand
    return None


def _resolve_penalty_value(values, penalty):
    if penalty is not None and math.isfinite(penalty):
        return float(penalty)
    finite = [v for v in values if math.isfinite(v)]
    if finite:
        return float(max(finite))
    return 0.0


def _apply_pose_error_mode(values, mode, plot_max, penalty):
    if mode == "finite":
        return [v for v in values if math.isfinite(v)]
    if mode == "original":
        fill = _resolve_penalty_value(values, plot_max)
        return [v if math.isfinite(v) else fill for v in values]
    if mode == "penalized":
        fill = _resolve_penalty_value(values, penalty)
        return [v if math.isfinite(v) else fill for v in values]
    return [v for v in values if math.isfinite(v)]


def _load_pose_errors_rows(path: Path, max_trans_e_m, max_rot_e_deg):
    te_vals = []
    re_vals = []
    total = 0
    nan_count = 0
    with path.open("r", encoding="utf-8") as handle:
        for line_idx, line in enumerate(handle):
            if line_idx == 0:
                continue
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            total += 1
            te_raw = float("nan") if parts[1] == "nan" else float(parts[1])
            re_raw = float("nan") if parts[2] == "nan" else float(parts[2])
            if math.isfinite(max_trans_e_m) and math.isfinite(te_raw) and te_raw > max_trans_e_m:
                te_raw = float("nan")
            if math.isfinite(max_rot_e_deg) and math.isfinite(re_raw) and re_raw > max_rot_e_deg:
                re_raw = float("nan")
            if not math.isfinite(te_raw) or not math.isfinite(re_raw):
                nan_count += 1
            te_vals.append(te_raw)
            re_vals.append(re_raw)
    return te_vals, re_vals, nan_count, total


def _resolve_base_cfg(args):
    cfg_path = getattr(args, "base_analysis_cfg", None)
    if not cfg_path:
        return {}
    path = Path(cfg_path)
    if not path.exists():
        return {}
    try:
        return load_yaml_config(path)
    except Exception:
        return {}


def parse_pose_errors(path: Path, stat: str, mode: str, base_cfg=None):
    if base_cfg is None:
        base_cfg = {}
    ana_cfg = dict(base_cfg)
    analysis_cfg = _find_analysis_cfg(path.parent)
    if analysis_cfg:
        try:
            ana_cfg.update(load_yaml_config(analysis_cfg))
        except Exception:
            pass
    hist_max_trans = float(ana_cfg.get("hist_max_trans_e", float("nan")))
    hist_max_rot = float(ana_cfg.get("hist_max_rot_e", float("nan")))
    max_trans_e_m = float(ana_cfg.get("max_trans_e_m", hist_max_trans))
    max_rot_e_deg = float(ana_cfg.get("max_rot_e_deg", hist_max_rot))

    te_raw, re_raw, nan_count, total = _load_pose_errors_rows(
        path, max_trans_e_m, max_rot_e_deg
    )
    plot_max_trans = 1.2 * hist_max_trans if math.isfinite(hist_max_trans) else float("nan")
    plot_max_rot = 1.2 * hist_max_rot if math.isfinite(hist_max_rot) else float("nan")
    penalty_trans = max_trans_e_m if math.isfinite(max_trans_e_m) else hist_max_trans
    penalty_rot = max_rot_e_deg if math.isfinite(max_rot_e_deg) else hist_max_rot

    te_vals = _apply_pose_error_mode(te_raw, mode, plot_max_trans, penalty_trans)
    re_vals = _apply_pose_error_mode(re_raw, mode, plot_max_rot, penalty_rot)
    if not te_vals or not re_vals:
        raise RuntimeError(f"pose_errors file has no usable entries: {path}")
    nan_ratio = float("nan") if total == 0 else nan_count / total
    return compute_stat(te_vals, stat), compute_stat(re_vals, stat), nan_ratio


def parse_pose_errors_files(paths, stat: str, mode: str, base_cfg=None):
    if base_cfg is None:
        base_cfg = {}
    te_vals = []
    re_vals = []
    nan_total = 0
    total = 0
    for path in paths:
        path_obj = Path(path)
        if not path_obj.exists():
            alt = None
            if path_obj.name == "pose_errors_path_yaw.txt":
                alt = path_obj.with_name("pose_errors.txt")
            elif path_obj.name == "pose_errors.txt":
                alt = path_obj.with_name("pose_errors_path_yaw.txt")
            if alt is not None and alt.exists():
                path_obj = alt
            else:
                raise RuntimeError(f"pose_errors file missing: {path_obj}")

        ana_cfg = dict(base_cfg)
        analysis_cfg = _find_analysis_cfg(path_obj.parent)
        if analysis_cfg:
            try:
                ana_cfg.update(load_yaml_config(analysis_cfg))
            except Exception:
                pass
        hist_max_trans = float(ana_cfg.get("hist_max_trans_e", float("nan")))
        hist_max_rot = float(ana_cfg.get("hist_max_rot_e", float("nan")))
        max_trans_e_m = float(ana_cfg.get("max_trans_e_m", hist_max_trans))
        max_rot_e_deg = float(ana_cfg.get("max_rot_e_deg", hist_max_rot))
        plot_max_trans = 1.2 * hist_max_trans if math.isfinite(hist_max_trans) else float("nan")
        plot_max_rot = 1.2 * hist_max_rot if math.isfinite(hist_max_rot) else float("nan")
        penalty_trans = max_trans_e_m if math.isfinite(max_trans_e_m) else hist_max_trans
        penalty_rot = max_rot_e_deg if math.isfinite(max_rot_e_deg) else hist_max_rot

        te_raw, re_raw, nan_count, total_count = _load_pose_errors_rows(
            path_obj, max_trans_e_m, max_rot_e_deg
        )
        te_vals.extend(_apply_pose_error_mode(te_raw, mode, plot_max_trans, penalty_trans))
        re_vals.extend(_apply_pose_error_mode(re_raw, mode, plot_max_rot, penalty_rot))
        nan_total += nan_count
        total += total_count
    if not te_vals or not re_vals:
        raise RuntimeError("pose_errors files have no usable entries.")
    nan_ratio = float("nan") if total == 0 else nan_total / total
    return compute_stat(te_vals, stat), compute_stat(re_vals, stat), nan_ratio


def compute_stat(values, stat):
    if stat == "min":
        return min(values)
    if stat == "max":
        return max(values)
    if stat == "std":
        return statistics.stdev(values)
    return statistics.mean(values)


def parse_schedule_last_alpha(schedule_str):
    if not schedule_str:
        return None
    parts = [p.strip() for p in schedule_str.split(",") if p.strip()]
    if not parts:
        return None
    try:
        return float(parts[-1])
    except ValueError:
        return None


def _resolve_ks_mode(mode):
    if isinstance(mode, (int, float)):
        return "const", float(mode)
    if not isinstance(mode, str):
        return None, None
    token = mode.strip().lower()
    if token in ("map", "mapped", "visibility", "from_visibility", "ks_from_visibility"):
        return "map", None
    if token.startswith("const"):
        token = token.replace("const", "", 1)
    try:
        return "const", float(token)
    except ValueError:
        return None, None


def build_search_space(trial, args):
    params = {}
    fixed = args.fixed_params or {}
    fixed_ks = fixed.get("ks")
    ks_mode_options = getattr(args, "ks_mode_options", None)
    if "max_iteration" in fixed:
        params["max_iteration"] = fixed["max_iteration"]
    elif args.tune_max_iteration:
        params["max_iteration"] = trial.suggest_int(
            "max_iteration", args.max_iteration_range[0], args.max_iteration_range[1]
        )
    if ks_mode_options:
        mode = trial.suggest_categorical("ks_mode", ks_mode_options)
        params["ks_mode"] = mode
        mode_kind, mode_val = _resolve_ks_mode(mode)
        if mode_kind == "const":
            params["ks_from_visibility"] = False
            params["ks"] = mode_val
        elif mode_kind == "map":
            params["ks_from_visibility"] = True
            if "ks_transition_deg" in fixed:
                params["ks_transition_deg"] = fixed["ks_transition_deg"]
            elif args.tune_ks_transition_deg:
                params["ks_transition_deg"] = trial.suggest_float(
                    "ks_transition_deg",
                    args.ks_transition_deg_range[0],
                    args.ks_transition_deg_range[1],
                )
            else:
                params["ks_transition_deg"] = args.ks_transition_deg
        else:
            raise SystemExit(f"Unknown ks_mode option: {mode}")
    else:
        if not args.ks_from_visibility:
            if fixed_ks is not None:
                params["ks"] = fixed_ks
            elif args.tune_ks:
                params["ks"] = trial.suggest_float(
                    "ks", args.ks_range[0], args.ks_range[1], log=args.ks_log
                )
    if "ks_transition_deg" in fixed:
        params["ks_transition_deg"] = fixed["ks_transition_deg"]
    elif args.tune_ks_transition_deg:
        params["ks_transition_deg"] = trial.suggest_float(
            "ks_transition_deg",
            args.ks_transition_deg_range[0],
            args.ks_transition_deg_range[1],
        )
    if "base_step_scale" in fixed:
        params["base_step_scale"] = fixed["base_step_scale"]
    elif args.tune_base_step:
        params["base_step_scale"] = trial.suggest_float(
            "base_step_scale",
            args.base_step_scale_range[0],
            args.base_step_scale_range[1],
            log=args.base_step_scale_log,
        )
    min_step_fixed = fixed.get("min_step_deg")
    max_step_fixed = fixed.get("max_step_deg")

    if min_step_fixed is not None:
        params["min_step_deg"] = float(min_step_fixed)
    if max_step_fixed is not None:
        params["max_step_deg"] = float(max_step_fixed)

    if "min_step_deg" not in params:
        if args.tune_min_step_deg:
            min_lo, min_hi = args.min_step_deg_range
            # If max is fixed, keep sampled min within a valid interval.
            if "max_step_deg" in params:
                min_hi = min(min_hi, params["max_step_deg"])
            if min_lo > min_hi:
                raise SystemExit(
                    "min_step_deg_range has no feasible value with current max_step_deg."
                )
            params["min_step_deg"] = trial.suggest_float("min_step_deg", min_lo, min_hi)

    if "max_step_deg" not in params:
        if args.tune_max_step_deg:
            max_lo, max_hi = args.max_step_deg_range
            # Keep max >= chosen min when min is already resolved.
            if "min_step_deg" in params:
                max_lo = max(max_lo, params["min_step_deg"])
            if max_lo > max_hi:
                raise SystemExit(
                    "max_step_deg_range has no feasible value with current min_step_deg."
                )
            params["max_step_deg"] = trial.suggest_float("max_step_deg", max_lo, max_hi)

    if (
        "min_step_deg" in params
        and "max_step_deg" in params
        and params["min_step_deg"] > params["max_step_deg"]
    ):
        raise SystemExit("Resolved min_step_deg is greater than max_step_deg.")
    if "trajectory_jacobian_step" in fixed:
        params["trajectory_jacobian_step"] = fixed["trajectory_jacobian_step"]
    elif args.tune_trajectory_jacobian_step:
        params["trajectory_jacobian_step"] = trial.suggest_float(
            "trajectory_jacobian_step",
            args.trajectory_jacobian_step_range[0],
            args.trajectory_jacobian_step_range[1],
        )
    if "step_norm_mode" in fixed:
        params["step_norm_mode"] = fixed["step_norm_mode"]
    elif args.tune_step_norm_mode:
        params["step_norm_mode"] = trial.suggest_categorical(
            "step_norm_mode", args.step_norm_mode_options
        )
    if "fov_schedule" in fixed:
        params["fov_schedule"] = fixed["fov_schedule"]
    elif args.tune_fov_schedule:
        params["fov_schedule"] = trial.suggest_categorical(
            "fov_schedule", args.fov_schedule_options
        )
    ks_from_visibility = params.get("ks_from_visibility", args.ks_from_visibility)
    if ks_from_visibility:
        alpha = None
        if "fov_schedule" in params:
            alpha = parse_schedule_last_alpha(params["fov_schedule"])
        ks_transition_deg = params.get("ks_transition_deg", args.ks_transition_deg)
        if alpha is not None and ks_transition_deg:
            ks_suggested = suggest_ks(alpha, ks_transition_deg)
            if ks_suggested is not None:
                if args.tune_ks and not ks_mode_options:
                    mults = args.ks_visibility_range_multipliers
                    if (
                        isinstance(mults, (list, tuple))
                        and len(mults) == 2
                        and mults[0] > 0
                        and mults[1] > 0
                    ):
                        params["ks"] = trial.suggest_float(
                            "ks",
                            ks_suggested * mults[0],
                            ks_suggested * mults[1],
                            log=args.ks_log,
                        )
                else:
                    params["ks"] = ks_suggested
        if ks_transition_deg is not None:
            params.setdefault("ks_transition_deg", ks_transition_deg)
        if alpha is not None:
            params["visibility_alpha_deg"] = alpha
    return params


def make_env(params, base_env, args=None):
    env = dict(base_env)
    if "max_iteration" in params:
        env["FOV_OPT_MAX_ITERATION"] = str(params["max_iteration"])
    if "ks" in params:
        env["FOV_OPT_KS"] = f"{params['ks']:.8f}"
    if "base_step_scale" in params:
        env["FOV_OPT_BASE_STEP_SCALE"] = f"{params['base_step_scale']:.8f}"
    if "min_step_deg" in params:
        env["FOV_OPT_MIN_STEP_DEG"] = f"{params['min_step_deg']:.8f}"
    if "max_step_deg" in params:
        env["FOV_OPT_MAX_STEP_DEG"] = f"{params['max_step_deg']:.8f}"
    if "step_norm_mode" in params:
        env["FOV_OPT_STEP_NORM_MODE"] = str(params["step_norm_mode"])
    if "trajectory_jacobian_step" in params:
        env["FOV_OPT_TRAJECTORY_JACOBIAN_STEP"] = (
            f"{params['trajectory_jacobian_step']:.8f}"
        )
    if "fov_schedule" in params:
        env["FOV_OPT_FOV_SCHEDULE"] = params["fov_schedule"]
    ks_from_visibility = None
    if "ks_from_visibility" in params:
        ks_from_visibility = bool(params["ks_from_visibility"])
    elif args is not None and getattr(args, "ks_from_visibility", False):
        ks_from_visibility = True
    if ks_from_visibility:
        env["FOV_OPT_KS_FROM_VISIBILITY"] = "1"
        ks_transition = params.get("ks_transition_deg", getattr(args, "ks_transition_deg", None))
        if ks_transition is not None:
            env["FOV_OPT_KS_TRANSITION_DEG"] = f"{float(ks_transition):.8f}"
    if args_warm_start_enabled := env.get("_FOV_OPT_WARM_START"):
        env["FOV_OPT_WARM_START"] = args_warm_start_enabled
    if warm_start_file := env.get("_FOV_OPT_WARM_START_FILE"):
        env["FOV_OPT_WARM_START_FILE"] = warm_start_file
    if args is not None and getattr(args, "vis_weight", 0.0):
        # Keep per-iteration quiver metrics so visibility improvement can be computed.
        env["FOV_OPT_KEEP_METRICS"] = "1"
    return env


def run_trial_command(cmd, workdir, env, quiet):
    if quiet:
        with open(os.devnull, "w") as devnull:
            subprocess.run(
                cmd, shell=True, check=True, cwd=workdir, env=env,
                stdout=devnull, stderr=devnull
            )
    else:
        subprocess.run(cmd, shell=True, check=True, cwd=workdir, env=env)


def _with_reg_image_limit(cmd, args):
    max_imgs = int(getattr(args, "max_reg_images_per_trial", 0) or 0)
    if max_imgs <= 0:
        return cmd
    if "run_planner_exp.py" not in cmd:
        return cmd
    # Apply per registration command when chained with &&.
    parts = [p.strip() for p in cmd.split("&&")]
    updated = []
    for part in parts:
        if "run_planner_exp.py" in part and "--max_reg_images" not in part:
            part = "{} --max_reg_images {}".format(part, max_imgs)
        updated.append(part)
    return " && ".join(updated)


def _with_render_options(cmd, args):
    if getattr(args, "mode", "") != "tune":
        return cmd
    if "run_planner_exp.py" not in cmd:
        return cmd
    width = getattr(args, "render_width_tune", None)
    height = getattr(args, "render_height_tune", None)
    fov = getattr(args, "render_fov_tune", None)
    sleep = getattr(args, "render_sleep_tune", None)
    if width:
        cmd += " --render_width {}".format(int(width))
    if height:
        cmd += " --render_height {}".format(int(height))
    if fov:
        cmd += " --render_fov {}".format(float(fov))
    if sleep is not None:
        cmd += " --render_sleep {}".format(float(sleep))
    return cmd


def _rewrite_trace_paths(paths, trace_root):
    if not paths or not trace_root:
        return paths
    resolved = []
    for path in paths:
        try:
            parts = Path(path).parts
            if "trace" in parts:
                idx = parts.index("trace")
                rel = Path(*parts[idx + 1:]) if idx + 1 < len(parts) else Path()
                resolved.append(str(Path(trace_root) / rel))
            else:
                resolved.append(path)
        except Exception:
            resolved.append(path)
    return resolved


def _extract_run_planner_config(tokens):
    for idx, tok in enumerate(tokens):
        if tok.endswith("run_planner_exp.py") and idx + 1 < len(tokens):
            cand = tokens[idx + 1]
            if not cand.startswith("-"):
                return cand
    return None


def _infer_base_names(config_path):
    if not config_path:
        return []
    try:
        cfg = load_yaml_config(Path(config_path))
    except Exception:
        return []
    names = []
    for cfg_val in cfg.values():
        if not isinstance(cfg_val, dict):
            continue
        base = cfg_val.get("base")
        if isinstance(base, dict):
            names.extend(list(base.values()))
    return names


def _augment_reg_cmd_with_trial_paths(cmd, trace_root):
    if not trace_root or "run_planner_exp.py" not in cmd:
        return cmd
    try:
        tokens = shlex.split(cmd)
    except ValueError:
        return cmd
    if "--top_outdir" not in tokens:
        tokens += ["--top_outdir", str(trace_root)]
    if "--optimized_dir" not in tokens and "--optimized_dirs" not in tokens:
        config_path = _extract_run_planner_config(tokens)
        base_names = _infer_base_names(config_path)
        if base_names:
            opt_specs = []
            for name in base_names:
                opt_dir = Path(trace_root) / name / f"{name}_none" / "optimized_path_yaw"
                opt_specs.append(f"{name}={opt_dir}")
            tokens += ["--optimized_dirs"] + opt_specs
    return " ".join(shlex.quote(tok) for tok in tokens)


def run_trial_pipeline(trial_id, args, env, workdir):
    times = {
        "optimization_sec": None,
        "registration_sec": None,
        "pipeline_sec": None,
    }
    pipeline_start = time.time()
    has_split_cmd = bool(
        args.run_cmd_optimization
        or args.run_cmd_registration_along_path
        or args.run_cmd_registration_normal
    )
    if not has_split_cmd:
        if args.trial_log:
            print("[trial {}] run pipeline".format(trial_id))
        run_trial_command(args.run_cmd, workdir, env, args.quiet_subprocess)
        times["pipeline_sec"] = time.time() - pipeline_start
        if args.trial_log:
            print("[trial {}] pipeline done ({:.2f}s)".format(
                trial_id, times["pipeline_sec"]))
        return times

    reg_cmd = (
        args.run_cmd_registration_along_path
        if args.along_path
        else args.run_cmd_registration_normal
    )
    trace_root = env.get("FOV_TRACE_ROOT") if env else None
    if trace_root:
        reg_cmd = _augment_reg_cmd_with_trial_paths(reg_cmd, trace_root)
    reg_cmd = _with_reg_image_limit(reg_cmd, args)
    reg_cmd = _with_render_options(reg_cmd, args)
    if args.trial_log:
        print("[trial {}] optimization start".format(trial_id))
    opt_start = time.time()
    run_trial_command(args.run_cmd_optimization, workdir, env, args.quiet_subprocess)
    times["optimization_sec"] = time.time() - opt_start
    if args.trial_log:
        print("[trial {}] optimization done ({:.2f}s)".format(
            trial_id, times["optimization_sec"]))
    if args.trial_log:
        max_imgs = int(getattr(args, "max_reg_images_per_trial", 0) or 0)
        if max_imgs > 0:
            print("[trial {}] registration start (max_reg_images={})".format(
                trial_id, max_imgs))
        else:
            print("[trial {}] registration start".format(trial_id))
    reg_start = time.time()
    run_trial_command(reg_cmd, workdir, env, args.quiet_subprocess)
    times["registration_sec"] = time.time() - reg_start
    times["pipeline_sec"] = time.time() - pipeline_start
    if args.trial_log:
        print("[trial {}] registration done ({:.2f}s)".format(
            trial_id, times["registration_sec"]))
        print("[trial {}] pipeline done ({:.2f}s)".format(
            trial_id, times["pipeline_sec"]))
    return times


def load_metrics(args, error_stat_path, pose_errors_path,
                 error_stat_files=None, pose_errors_files=None):
    mode = getattr(args, "pose_error_mode", "finite")
    if mode not in POSE_ERROR_MODES:
        mode = "finite"
    base_cfg = _resolve_base_cfg(args)
    if error_stat_files is None:
        error_stat_files = args.error_stat_files
    if pose_errors_files is None:
        pose_errors_files = args.pose_errors_files
    if error_stat_files:
        if mode != "finite":
            print(
                "Warning: pose_error_mode is {}, but error_stat files do not support penalized/original handling. "
                "Using error_stat as-is.".format(mode),
                file=sys.stderr,
            )
        te, re = parse_error_stat_files(error_stat_files, args.stat)
        return te, re, float("nan")
    if pose_errors_files:
        return parse_pose_errors_files(pose_errors_files, args.stat, mode, base_cfg)
    if error_stat_path and error_stat_path.exists():
        if mode != "finite":
            print(
                "Warning: pose_error_mode is {}, but error_stat does not support penalized/original handling. "
                "Using error_stat as-is.".format(mode),
                file=sys.stderr,
            )
        te, re = parse_error_stat(error_stat_path, args.stat)
        return te, re, float("nan")
    if pose_errors_path and pose_errors_path.exists():
        return parse_pose_errors(pose_errors_path, args.stat, mode, base_cfg)
    raise RuntimeError("No error file found after run.")


def _parse_quiver_metrics(path: Path, metric: str):
    # Each non-empty line: x,y,z,dx,dy,dz,vis_count,vis_score
    # Empty lines separate iterations.
    values = []
    cur = []
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                if cur:
                    values.append(sum(cur) / len(cur))
                    cur = []
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) < 8:
                continue
            try:
                vis_count = float(parts[6])
                vis_score = float(parts[7])
            except ValueError:
                continue
            if metric == "vis30_score":
                cur.append(vis_score)
            else:
                cur.append(vis_count)
    if cur:
        values.append(sum(cur) / len(cur))
    return values


def compute_visibility_improvement(roots, pattern, metric, mode):
    if not roots:
        return None
    if isinstance(roots, (str, Path)):
        roots = [roots]
    improvements = []
    for root in roots:
        root_path = Path(root).expanduser().resolve()
        if not root_path.exists():
            continue
        for metrics_path in root_path.rglob(pattern):
            try:
                series = _parse_quiver_metrics(metrics_path, metric)
            except OSError:
                continue
            if len(series) < 2:
                continue
            first = series[0]
            last = series[-1]
            if mode == "ratio":
                denom = first if abs(first) > 1e-6 else 1.0
                improvements.append((last - first) / denom)
            else:
                improvements.append(last - first)
    if not improvements:
        return None
    return sum(improvements) / len(improvements)


def resolve_vis_roots(args, env):
    env_root = None
    if env:
        env_root = env.get("FOV_TRACE_ROOT")
    if env_root:
        return [env_root]
    roots = getattr(args, "vis_trace_root", None)
    if not roots:
        return []
    if isinstance(roots, (str, Path)):
        return [roots]
    return list(roots)


def load_yaml_config(path: Path):
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "PyYAML is required to read the config file. Install with: pip install pyyaml"
        ) from exc
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"Config file must be a mapping: {path}")
    return data


def write_yaml_config(path: Path, data):
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "PyYAML is required to write the output files. Install with: pip install pyyaml"
        ) from exc
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False)


def _write_trials_csv(path: Path, trials):
    rows = []
    param_keys = set()
    for t in trials:
        params = t.user_attrs.get("resolved_params")
        if not isinstance(params, dict):
            params = t.params or {}
        param_keys.update(params.keys())
    param_keys = sorted(param_keys)
    for t in trials:
        params = t.user_attrs.get("resolved_params")
        if not isinstance(params, dict):
            params = t.params or {}
        row = {k: params.get(k, "") for k in param_keys}
        row["te"] = t.user_attrs.get("te")
        row["re"] = t.user_attrs.get("re")
        row["vis_improvement"] = t.user_attrs.get("vis_improvement")
        rows.append(row)
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = param_keys + ["te", "re", "vis_improvement"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def normalize_fixed_params(raw):
    if not isinstance(raw, dict):
        return {}
    raw = _apply_legacy_aliases_dict(raw)
    return {k: v for k, v in raw.items() if k in SUPPORTED_FIXED_PARAMS}


def suggest_ks(alpha_deg: float, transition_deg: float):
    if alpha_deg <= 0.0 or transition_deg <= 0.0:
        return None
    alpha_rad = math.radians(alpha_deg)
    delta_rad = math.radians(transition_deg)
    sin_alpha = math.sin(alpha_rad)
    if abs(sin_alpha) < 1e-6 or delta_rad <= 0.0:
        return None
    # Approximate 5%->95% transition width around alpha.
    return 2.94 / (delta_rad * sin_alpha)


def validate_config(args):
    missing = []
    fixed = args.fixed_params or {}
    mode = getattr(args, "mode", None)
    pose_mode = getattr(args, "pose_error_mode", "finite")
    if pose_mode not in POSE_ERROR_MODES:
        raise SystemExit(
            f"pose_error_mode must be one of {sorted(POSE_ERROR_MODES)} (got {pose_mode})."
        )

    def require(name):
        if not hasattr(args, name) or getattr(args, name) is None:
            missing.append(name)

    for key in (
        "mode",
        "along_path",
        "trial_log",
        "quiet_subprocess",
        "optuna_log_level",
        "n_jobs",
        "run_cmd",
        "run_cmd_optimization",
        "run_cmd_registration_along_path",
        "run_cmd_registration_normal",
        "workdir",
        "stat",
        "n_trials",
        "timeout",
        "study_name",
        "storage",
        "seed",
        "multi_objective",
        "te_weight",
        "re_weight",
        "trial_dir_root",
        "log_jacobian",
        "tune_max_iteration",
        "tune_ks",
        "tune_ks_transition_deg",
        "tune_base_step",
        "tune_min_step_deg",
        "tune_max_step_deg",
        "tune_trajectory_jacobian_step",
        "tune_step_norm_mode",
        "tune_fov_schedule",
        "ks_from_visibility",
        "fixed_params",
        "warm_start",
        "warm_start_file",
        "pose_errors_files",
        "error_stat_files",
        "results_jsonl",
        "best_params_yaml",
        "pareto_params_yaml",
        "run_cmd_along_path",
        "run_cmd_normal",
        "pose_errors_files_along_path",
        "pose_errors_files_normal",
        "error_stat_files_along_path",
        "error_stat_files_normal",
    ):
        require(key)

    if mode == "fixed":
        supported = SUPPORTED_FIXED_PARAMS
        if not isinstance(fixed, dict) or not any(k in fixed for k in supported):
            missing.append("fixed_params (with at least one supported key)")

    if args.tune_max_iteration and "max_iteration" not in fixed:
        require("max_iteration_range")
    if args.tune_base_step and "base_step_scale" not in fixed:
        require("base_step_scale_range")
        require("base_step_scale_log")
    if args.tune_min_step_deg and "min_step_deg" not in fixed:
        require("min_step_deg_range")
    if args.tune_max_step_deg and "max_step_deg" not in fixed:
        require("max_step_deg_range")
    if args.tune_trajectory_jacobian_step and "trajectory_jacobian_step" not in fixed:
        require("trajectory_jacobian_step_range")
    if args.tune_step_norm_mode and "step_norm_mode" not in fixed:
        require("step_norm_mode_options")
    if args.tune_fov_schedule and "fov_schedule" not in fixed:
        require("fov_schedule_options")

    ks_mode_options = getattr(args, "ks_mode_options", None)
    if ks_mode_options:
        if not isinstance(ks_mode_options, (list, tuple)) or not ks_mode_options:
            raise SystemExit("ks_mode_options must be a non-empty list.")
        if any(_resolve_ks_mode(opt)[0] == "map" for opt in ks_mode_options):
            require("ks_transition_deg")
    else:
        if args.tune_ks and "ks" not in fixed:
            require("ks_log")
            if not args.ks_from_visibility:
                require("ks_range")
    if args.tune_ks_transition_deg and "ks_transition_deg" not in fixed:
        require("ks_transition_deg_range")
    if args.ks_from_visibility:
        require("ks_transition_deg")
        require("ks_visibility_range_multipliers")
        has_alpha = False
        if args.tune_fov_schedule and "fov_schedule" not in fixed:
            has_alpha = True
        if "fov_schedule" in fixed:
            has_alpha = True
        if not has_alpha:
            missing.append("fov_schedule (for ks_from_visibility)")

    has_split_cmd = bool(args.run_cmd_optimization or args.run_cmd_registration_along_path or args.run_cmd_registration_normal)
    if has_split_cmd:
        if not args.run_cmd_optimization:
            missing.append("run_cmd_optimization")
        if args.along_path and not args.run_cmd_registration_along_path:
            missing.append("run_cmd_registration_along_path")
        if (not args.along_path) and not args.run_cmd_registration_normal:
            missing.append("run_cmd_registration_normal")
    elif not args.run_cmd:
        missing.append("run_cmd (selected from run_cmd_along_path/run_cmd_normal)")

    if missing:
        raise SystemExit(f"Missing config keys: {', '.join(sorted(set(missing)))}")

    fixed_min = fixed.get("min_step_deg")
    fixed_max = fixed.get("max_step_deg")
    if fixed_min is not None and fixed_max is not None and fixed_min > fixed_max:
        raise SystemExit("fixed_params min_step_deg must be <= max_step_deg.")
    if args.tune_min_step_deg and args.tune_max_step_deg:
        if args.min_step_deg_range[0] > args.max_step_deg_range[1]:
            raise SystemExit(
                "min_step_deg_range lower bound cannot be greater than max_step_deg_range upper bound."
            )

    if args.fixed_params is not None and not isinstance(args.fixed_params, dict):
        raise SystemExit("fixed_params must be a mapping of param names to values.")
    if args.initial_params is not None and not isinstance(args.initial_params, dict):
        raise SystemExit("initial_params must be a mapping of param names to values.")
    if args.ks_from_visibility and isinstance(fixed, dict) and "ks" in fixed:
        print(
            "Warning: ks_from_visibility is enabled; fixed_params['ks'] will be ignored.",
            file=sys.stderr,
        )
    if getattr(args, "ks_mode_options", None) and isinstance(fixed, dict) and "ks" in fixed:
        print(
            "Warning: ks_mode_options is enabled; fixed_params['ks'] will be ignored.",
            file=sys.stderr,
        )


def _tunable_keys(args, fixed):
    keys = []
    if args.tune_max_iteration and "max_iteration" not in fixed:
        keys.append("max_iteration")
    if getattr(args, "ks_mode_options", None):
        keys.append("ks_mode")
    elif args.tune_ks and "ks" not in fixed:
        keys.append("ks")
    if args.tune_ks_transition_deg and "ks_transition_deg" not in fixed:
        keys.append("ks_transition_deg")
    if args.tune_base_step and "base_step_scale" not in fixed:
        keys.append("base_step_scale")
    if args.tune_min_step_deg and "min_step_deg" not in fixed:
        keys.append("min_step_deg")
    if args.tune_max_step_deg and "max_step_deg" not in fixed:
        keys.append("max_step_deg")
    if args.tune_trajectory_jacobian_step and "trajectory_jacobian_step" not in fixed:
        keys.append("trajectory_jacobian_step")
    if args.tune_step_norm_mode and "step_norm_mode" not in fixed:
        keys.append("step_norm_mode")
    if args.tune_fov_schedule and "fov_schedule" not in fixed:
        keys.append("fov_schedule")
    return keys


def _validate_initial_params(args, fixed):
    if not args.enqueue_initial or not args.initial_params:
        return {}
    tunable = set(_tunable_keys(args, fixed))
    initial = {k: v for k, v in args.initial_params.items() if k in tunable}
    if not initial:
        return {}
    # Range checks for numeric params.
    def in_range(name, value, rmin, rmax):
        if value < rmin or value > rmax:
            raise SystemExit(
                f"initial_params[{name}]={value} out of range [{rmin}, {rmax}]"
            )
    max_iteration_range = getattr(args, "max_iteration_range", None)
    ks_range = getattr(args, "ks_range", None)
    ks_transition_deg_range = getattr(args, "ks_transition_deg_range", None)
    base_step_scale_range = getattr(args, "base_step_scale_range", None)
    min_step_deg_range = getattr(args, "min_step_deg_range", None)
    max_step_deg_range = getattr(args, "max_step_deg_range", None)
    trajectory_jacobian_step_range = getattr(args, "trajectory_jacobian_step_range", None)
    step_norm_mode_options = getattr(args, "step_norm_mode_options", None)
    fov_schedule_options = getattr(args, "fov_schedule_options", None)

    if "max_iteration" in initial and max_iteration_range:
        in_range("max_iteration", initial["max_iteration"], max_iteration_range[0], max_iteration_range[1])
    if "ks" in initial and ks_range and not args.ks_from_visibility:
        in_range("ks", initial["ks"], ks_range[0], ks_range[1])
    if "ks_transition_deg" in initial and ks_transition_deg_range:
        in_range(
            "ks_transition_deg",
            initial["ks_transition_deg"],
            ks_transition_deg_range[0],
            ks_transition_deg_range[1],
        )
    if "base_step_scale" in initial and base_step_scale_range:
        in_range(
            "base_step_scale",
            initial["base_step_scale"],
            base_step_scale_range[0],
            base_step_scale_range[1],
        )
    if "min_step_deg" in initial and min_step_deg_range:
        in_range(
            "min_step_deg",
            initial["min_step_deg"],
            min_step_deg_range[0],
            min_step_deg_range[1],
        )
    if "max_step_deg" in initial and max_step_deg_range:
        in_range(
            "max_step_deg",
            initial["max_step_deg"],
            max_step_deg_range[0],
            max_step_deg_range[1],
        )
    if "trajectory_jacobian_step" in initial and trajectory_jacobian_step_range:
        in_range(
            "trajectory_jacobian_step",
            initial["trajectory_jacobian_step"],
            trajectory_jacobian_step_range[0],
            trajectory_jacobian_step_range[1],
        )
    if "step_norm_mode" in initial and step_norm_mode_options:
        if initial["step_norm_mode"] not in step_norm_mode_options:
            raise SystemExit(
                "initial_params[step_norm_mode] must be one of step_norm_mode_options"
            )
    if "fov_schedule" in initial and fov_schedule_options:
        if initial["fov_schedule"] not in fov_schedule_options:
            raise SystemExit(
                "initial_params[fov_schedule] must be one of fov_schedule_options"
            )
    if (
        "min_step_deg" in initial
        and "max_step_deg" in initial
        and initial["min_step_deg"] > initial["max_step_deg"]
    ):
        raise SystemExit("initial_params min_step_deg must be <= max_step_deg.")
    return initial


def _existing_categorical_choices(study, param_name):
    for trial in study.trials:
        dist = trial.distributions.get(param_name)
        if dist is None:
            continue
        if isinstance(dist, optuna.distributions.CategoricalDistribution):
            return list(dist.choices)
    return None


def _sync_categorical_options_with_study(study, args, param_name, options_attr, enabled):
    if not enabled:
        return
    options = getattr(args, options_attr, None)
    existing = _existing_categorical_choices(study, param_name)
    if not existing:
        return
    if options is None:
        setattr(args, options_attr, list(existing))
        print(
            f"Warning: Using existing study {param_name} options {existing} (config missing).",
            file=sys.stderr,
        )
        return
    if list(options) == list(existing):
        return
    try:
        if set(options) == set(existing):
            setattr(args, options_attr, list(existing))
            print(
                f"Warning: Reordered {param_name} options to match existing study: {existing}.",
                file=sys.stderr,
            )
            return
    except TypeError:
        pass
    raise SystemExit(
        f"Existing study uses {param_name} options {existing}, "
        f"but config provides {list(options)}. "
        "Use the same options/order or start a new study."
    )


def main():
    parser = argparse.ArgumentParser(description="Optuna tuning for FoV optimization.")
    parser.add_argument("--config", default="", help="Path to YAML config file.")
    parser.add_argument("--mode", choices=["tune", "fixed"], default=None)
    parser.add_argument("--along-path", action="store_true", default=None)
    parser.add_argument("--normal", action="store_false", dest="along_path", default=None,
                        help="force along_path false")
    parser.add_argument("--run-cmd", help="Command that runs the full pipeline.")
    parser.add_argument("--max-reg-images-per-trial", type=int,
                        help="Limit registration images per trial (0 uses all).")
    parser.add_argument("--workdir", help="Working directory for run-cmd.")
    parser.add_argument("--error-stat", help="Path to error_stat.txt.")
    parser.add_argument("--pose-errors", help="Path to pose_errors.txt.")
    parser.add_argument("--stat", choices=STAT_INDEX.keys())
    parser.add_argument("--n-trials", type=int)
    parser.add_argument("--timeout", type=int, help="Seconds before stopping.")
    parser.add_argument("--study-name")
    parser.add_argument("--storage")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--multi-objective", action="store_true", default=None)
    parser.add_argument("--te-weight", type=float)
    parser.add_argument("--re-weight", type=float)
    parser.add_argument("--vis-weight", type=float)
    parser.add_argument("--vis-metric", choices=["vis30_count", "vis30_score"])
    parser.add_argument("--vis-improvement", choices=["delta", "ratio"])
    parser.add_argument("--vis-trace-root", nargs="+")
    parser.add_argument("--vis-pattern")
    parser.add_argument("--results-jsonl")
    parser.add_argument("--results-csv")
    parser.add_argument("--best-params-yaml")
    parser.add_argument("--pareto-params-yaml")
    parser.add_argument("--trial-dir-root", help="Optional base dir for trial outputs.")
    parser.add_argument("--log-jacobian", type=int, choices=[0, 1])

    parser.add_argument("--ks-from-visibility", action="store_true", default=None)
    parser.add_argument("--no-ks-from-visibility", action="store_false", dest="ks_from_visibility", default=None)

    parser.add_argument(
        "--tune-max-iteration",
        "--tune-max-iter",
        dest="tune_max_iteration",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--max-iteration-range",
        "--max-iter-range",
        dest="max_iteration_range",
        type=int,
        nargs=2,
    )
    parser.add_argument("--tune-ks", action="store_true", default=None)
    parser.add_argument("--ks-range", type=float, nargs=2)
    parser.add_argument("--ks-log", action="store_true", default=None)
    parser.add_argument("--ks-mode-options", nargs="+")
    parser.add_argument("--tune-ks-transition-deg", action="store_true", default=None)
    parser.add_argument("--ks-transition-deg-range", type=float, nargs=2)
    parser.add_argument("--tune-base-step", action="store_true", default=None)
    parser.add_argument("--base-step-scale-range", type=float, nargs=2)
    parser.add_argument("--base-step-scale-log", action="store_true", default=None)
    parser.add_argument("--tune-min-step-deg", action="store_true", default=None)
    parser.add_argument("--min-step-deg-range", type=float, nargs=2)
    parser.add_argument("--tune-max-step-deg", action="store_true", default=None)
    parser.add_argument("--max-step-deg-range", type=float, nargs=2)
    parser.add_argument(
        "--tune-trajectory-jacobian-step",
        "--tune-traj-jac-step",
        dest="tune_trajectory_jacobian_step",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--trajectory-jacobian-step-range",
        "--traj-jac-step-range",
        dest="trajectory_jacobian_step_range",
        type=float,
        nargs=2,
    )
    parser.add_argument("--tune-step-norm-mode", action="store_true", default=None)
    parser.add_argument("--step-norm-mode-options", nargs="+")
    parser.add_argument("--tune-fov-schedule", action="store_true", default=None)
    parser.add_argument("--fov-schedule-options", nargs="+")

    cli_args = parser.parse_args()

    config_path = Path(cli_args.config) if cli_args.config else None
    if not config_path:
        config_path = Path(__file__).with_name("optuna_fov_tune.yaml")
    if not config_path.exists():
        raise SystemExit(f"Config file not found: {config_path}")

    config_data = load_yaml_config(config_path)
    merged = dict(config_data)
    for key, value in vars(cli_args).items():
        if key == "config":
            continue
        if value is not None:
            merged[key] = value

    merged = _apply_config_aliases(merged)

    args = argparse.Namespace(**merged)

    if args.along_path is None:
        args.along_path = False
    if not hasattr(args, "fixed_params") or args.fixed_params is None:
        args.fixed_params = {}

    if args.trial_log is None:
        args.trial_log = True
    if not hasattr(args, "render_width_tune") or args.render_width_tune is None:
        args.render_width_tune = None
    if not hasattr(args, "render_height_tune") or args.render_height_tune is None:
        args.render_height_tune = None
    if not hasattr(args, "render_fov_tune") or args.render_fov_tune is None:
        args.render_fov_tune = None
    if not hasattr(args, "render_sleep_tune") or args.render_sleep_tune is None:
        args.render_sleep_tune = None
    if not hasattr(args, "results_csv") or args.results_csv is None:
        args.results_csv = ""
    if not hasattr(args, "vis_weight") or args.vis_weight is None:
        args.vis_weight = 0.0
    if not hasattr(args, "vis_metric") or args.vis_metric is None:
        args.vis_metric = "vis30_count"
    if not hasattr(args, "vis_improvement") or args.vis_improvement is None:
        args.vis_improvement = "delta"
    if not hasattr(args, "vis_pattern") or args.vis_pattern is None:
        args.vis_pattern = "quivers_path_yaw_metrics.txt"
    if not hasattr(args, "pose_error_mode") or args.pose_error_mode is None:
        args.pose_error_mode = "finite"
    if not hasattr(args, "base_analysis_cfg") or args.base_analysis_cfg is None:
        args.base_analysis_cfg = str(
            Path(__file__).resolve().parents[1]
            / "act_map_exp"
            / "params"
            / "quad_rrt"
            / "base_analysis_cfg.yaml"
        )

    if not hasattr(args, "enqueue_initial") or args.enqueue_initial is None:
        args.enqueue_initial = False
    if not hasattr(args, "initial_params") or args.initial_params is None:
        args.initial_params = {}
    # Default registration-image budget by mode unless explicitly overridden via CLI.
    if cli_args.max_reg_images_per_trial is None:
        args.max_reg_images_per_trial = 10 if args.mode == "tune" else 0

    if args.optuna_log_level:
        level = getattr(optuna.logging, str(args.optuna_log_level).upper(), None)
        if level is None:
            raise SystemExit("Invalid optuna_log_level: {}".format(args.optuna_log_level))
        optuna.logging.set_verbosity(level)

    if args.along_path and args.run_cmd_along_path:
        args.run_cmd = args.run_cmd_along_path
    elif not args.along_path and args.run_cmd_normal:
        args.run_cmd = args.run_cmd_normal

    if args.along_path and args.pose_errors_files_along_path:
        args.pose_errors_files = args.pose_errors_files_along_path
    elif not args.along_path and args.pose_errors_files_normal:
        args.pose_errors_files = args.pose_errors_files_normal

    if args.along_path and args.error_stat_files_along_path:
        args.error_stat_files = args.error_stat_files_along_path
    elif not args.along_path and args.error_stat_files_normal:
        args.error_stat_files = args.error_stat_files_normal

    if (
        not args.error_stat
        and not args.pose_errors
        and not args.pose_errors_files
        and not args.error_stat_files
    ):
        raise SystemExit(
            "Provide error_stat, pose_errors, pose_errors_files, or error_stat_files in the YAML config."
        )

    # In fixed mode, allow running directly from best_params_yaml without manual copy.
    if args.mode == "fixed":
        current_fixed = getattr(args, "fixed_params", None)
        if isinstance(current_fixed, dict) and current_fixed:
            args.fixed_params = normalize_fixed_params(current_fixed)
        else:
            best_yaml = getattr(args, "best_params_yaml", None)
            if best_yaml and Path(best_yaml).exists():
                best_data = load_yaml_config(Path(best_yaml))
                args.fixed_params = normalize_fixed_params(best_data.get("fixed_params"))

    if args.mode == "fixed":
        args.tune_max_iteration = False
        args.tune_ks = False
        args.tune_ks_transition_deg = False
        args.tune_base_step = False
        args.tune_min_step_deg = False
        args.tune_max_step_deg = False
        args.tune_trajectory_jacobian_step = False
        args.tune_step_norm_mode = False
        args.tune_fov_schedule = False

    validate_config(args)

    workdir = Path(args.workdir).resolve()
    error_stat_path = Path(args.error_stat).resolve() if args.error_stat else None
    pose_errors_path = Path(args.pose_errors).resolve() if args.pose_errors else None
    trial_root = Path(args.trial_dir_root).resolve() if args.trial_dir_root else None

    if args.mode == "fixed":
        params = build_search_space(None, args)
        base_env = dict(os.environ)
        base_env["_FOV_OPT_WARM_START"] = "1" if args.warm_start else "0"
        if args.warm_start_file:
            base_env["_FOV_OPT_WARM_START_FILE"] = args.warm_start_file
        env = make_env(params, base_env, args)
        env["FOV_OPT_LOG_JACOBIAN"] = str(args.log_jacobian)

        if trial_root:
            fixed_dir = trial_root / "fixed_run"
            fixed_dir.mkdir(parents=True, exist_ok=True)
            env["FOV_OPT_TRIAL_DIR"] = str(fixed_dir)
            trace_root = fixed_dir / "trace"
            trace_root.mkdir(parents=True, exist_ok=True)
            env["FOV_TRACE_ROOT"] = str(trace_root)

        start = time.time()
        if args.trial_log:
            print("[fixed] start")
        pipeline_times = run_trial_pipeline("fixed", args, env, workdir)
        trace_root = env.get("FOV_TRACE_ROOT")
        error_stat_files = _rewrite_trace_paths(args.error_stat_files, trace_root)
        pose_errors_files = _rewrite_trace_paths(args.pose_errors_files, trace_root)
        if error_stat_path and trace_root:
            error_stat_path = Path(_rewrite_trace_paths([str(error_stat_path)], trace_root)[0])
        if pose_errors_path and trace_root:
            pose_errors_path = Path(_rewrite_trace_paths([str(pose_errors_path)], trace_root)[0])
        te, re, nan_ratio = load_metrics(
            args,
            error_stat_path,
            pose_errors_path,
            error_stat_files=error_stat_files,
            pose_errors_files=pose_errors_files,
        )
        vis_improvement = None
        if args.vis_weight:
            roots = resolve_vis_roots(args, env)
            vis_improvement = compute_visibility_improvement(
                roots, args.vis_pattern, args.vis_metric, args.vis_improvement
            )
            if vis_improvement is None:
                vis_improvement = 0.0
        elapsed = time.time() - start
        if pipeline_times and pipeline_times.get("pipeline_sec") is not None:
            elapsed = pipeline_times["pipeline_sec"]
        objective = te if args.multi_objective else (args.te_weight * te + args.re_weight * re - args.vis_weight * (vis_improvement or 0.0))

        if args.trial_log:
            print("[fixed] obj={} te={} re={} elapsed={:.2f}s".format(
                "{:.6f}".format(objective) if not args.multi_objective else "(multi)",
                "{:.6f}".format(te),
                "{:.6f}".format(re),
                elapsed,
            ))
            if math.isfinite(nan_ratio):
                print("[fixed] nan_ratio={:.3f}".format(nan_ratio))
            if args.vis_weight:
                print("[fixed] vis_improvement={:.6f}".format(vis_improvement or 0.0))
            print("[fixed] params {}".format(params))

        if args.results_jsonl:
            results_path = Path(args.results_jsonl)
            record = {
                "mode": "fixed",
                "state": "complete",
                "value": objective if not args.multi_objective else None,
                "values": [te, re] if args.multi_objective else None,
                "params": params,
                "user_attrs": {
                    "te": te,
                    "re": re,
                    "vis_improvement": vis_improvement,
                    "elapsed_sec": elapsed,
                    "optimization_sec": (pipeline_times or {}).get("optimization_sec"),
                    "registration_sec": (pipeline_times or {}).get("registration_sec"),
                    "pipeline_sec": (pipeline_times or {}).get("pipeline_sec"),
                },
            }
            with results_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record) + "\n")
        if args.best_params_yaml:
            payload = {
                "mode": "fixed",
                "fixed_params": normalize_fixed_params(params),
                "best_value": objective if not args.multi_objective else None,
                "metrics": {"te": te, "re": re, "vis_improvement": vis_improvement, "nan_ratio": nan_ratio},
                "study_name": args.study_name,
            }
            write_yaml_config(Path(args.best_params_yaml), payload)
        return

    if args.storage:
        storage = args.storage
    else:
        storage = None

    if args.multi_objective:
        study = optuna.create_study(
            directions=["minimize", "minimize"],
            study_name=args.study_name,
            storage=storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=args.seed if args.seed else None),
        )
    else:
        study = optuna.create_study(
            direction="minimize",
            study_name=args.study_name,
            storage=storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=args.seed if args.seed else None),
        )

    fixed = args.fixed_params or {}
    _sync_categorical_options_with_study(
        study,
        args,
        "step_norm_mode",
        "step_norm_mode_options",
        enabled=bool(args.tune_step_norm_mode and "step_norm_mode" not in fixed),
    )
    _sync_categorical_options_with_study(
        study,
        args,
        "fov_schedule",
        "fov_schedule_options",
        enabled=bool(args.tune_fov_schedule and "fov_schedule" not in fixed),
    )
    _sync_categorical_options_with_study(
        study,
        args,
        "ks_mode",
        "ks_mode_options",
        enabled=bool(getattr(args, "ks_mode_options", None)),
    )

    initial = _validate_initial_params(args, fixed)
    if initial:
        study.enqueue_trial(initial)

    def objective(trial):
        params = build_search_space(trial, args)
        trial.set_user_attr("resolved_params", params)
        base_env = dict(os.environ)
        base_env["_FOV_OPT_WARM_START"] = "1" if args.warm_start else "0"
        if args.warm_start_file:
            base_env["_FOV_OPT_WARM_START_FILE"] = args.warm_start_file
        env = make_env(params, base_env, args)
        env["FOV_OPT_LOG_JACOBIAN"] = str(args.log_jacobian)

        trial_dir = None
        if trial_root:
            trial_dir = trial_root / f"trial_{trial.number:04d}"
            trial_dir.mkdir(parents=True, exist_ok=True)
            env["FOV_OPT_TRIAL_DIR"] = str(trial_dir)
            trace_root = trial_dir / "trace"
            trace_root.mkdir(parents=True, exist_ok=True)
            env["FOV_TRACE_ROOT"] = str(trace_root)

        start = time.time()
        if args.trial_log:
            print("[trial {}] start".format(trial.number))
        pipeline_times = run_trial_pipeline(trial.number, args, env, workdir)
        elapsed = time.time() - start
        if pipeline_times and pipeline_times.get("pipeline_sec") is not None:
            elapsed = pipeline_times["pipeline_sec"]
        trace_root = env.get("FOV_TRACE_ROOT")
        error_stat_files = _rewrite_trace_paths(args.error_stat_files, trace_root)
        pose_errors_files = _rewrite_trace_paths(args.pose_errors_files, trace_root)
        error_stat_path_local = error_stat_path
        pose_errors_path_local = pose_errors_path
        if error_stat_path_local and trace_root:
            error_stat_path_local = Path(
                _rewrite_trace_paths([str(error_stat_path_local)], trace_root)[0]
            )
        if pose_errors_path_local and trace_root:
            pose_errors_path_local = Path(
                _rewrite_trace_paths([str(pose_errors_path_local)], trace_root)[0]
            )

        te, re, nan_ratio = load_metrics(
            args,
            error_stat_path_local,
            pose_errors_path_local,
            error_stat_files=error_stat_files,
            pose_errors_files=pose_errors_files,
        )
        vis_improvement = None
        if args.vis_weight:
            roots = resolve_vis_roots(args, env)
            vis_improvement = compute_visibility_improvement(
                roots, args.vis_pattern, args.vis_metric, args.vis_improvement
            )
            if vis_improvement is None:
                vis_improvement = 0.0

        trial.set_user_attr("te", te)
        trial.set_user_attr("re", re)
        if math.isfinite(nan_ratio):
            trial.set_user_attr("nan_ratio", nan_ratio)
        trial.set_user_attr("vis_improvement", vis_improvement)
        trial.set_user_attr("elapsed_sec", elapsed)
        if pipeline_times:
            trial.set_user_attr("optimization_sec", pipeline_times.get("optimization_sec"))
            trial.set_user_attr("registration_sec", pipeline_times.get("registration_sec"))
            trial.set_user_attr("pipeline_sec", pipeline_times.get("pipeline_sec"))
        if trial_dir:
            trial.set_user_attr("trial_dir", str(trial_dir))

        trial.set_user_attr(
            "objective",
            te if args.multi_objective else (args.te_weight * te + args.re_weight * re - args.vis_weight * (vis_improvement or 0.0)),
        )
        if args.trial_log:
            current_obj = trial.user_attrs.get("objective")
            best_obj = None
            try:
                best_obj = study.best_value if not args.multi_objective else None
            except Exception:
                best_obj = None
            vis_alpha = params.get("visibility_alpha_deg")
            log_parts = [
                "[trial {}] obj={}".format(trial.number, "{:.6f}".format(current_obj) if current_obj is not None else "n/a"),
                "best={}".format("{:.6f}".format(best_obj) if best_obj is not None else "n/a"),
                "alpha={}".format("{:.3f}".format(vis_alpha) if vis_alpha is not None else "n/a"),
            ]
            print(" ".join(log_parts))
            if math.isfinite(nan_ratio):
                print("[trial {}] nan_ratio={:.3f}".format(trial.number, nan_ratio))
            if args.vis_weight:
                print("[trial {}] vis_improvement={:.6f}".format(trial.number, vis_improvement or 0.0))
            # Print key params in a compact one-line dict.
            compact = {
                k: params.get(k)
                for k in (
                    "max_iteration",
                    "ks",
                    "ks_transition_deg",
                    "base_step_scale",
                    "min_step_deg",
                    "max_step_deg",
                    "step_norm_mode",
                    "trajectory_jacobian_step",
                    "fov_schedule",
                )
                if k in params
            }
            print("[trial {}] params {}".format(trial.number, compact))
        if args.multi_objective:
            return te, re
        return args.te_weight * te + args.re_weight * re - args.vis_weight * (vis_improvement or 0.0)

    study.optimize(
        objective,
        n_trials=args.n_trials,
        timeout=args.timeout or None,
        n_jobs=args.n_jobs if args.n_jobs and args.n_jobs > 0 else 1,
    )

    if args.results_jsonl:
        results_path = Path(args.results_jsonl)
        with results_path.open("w", encoding="utf-8") as handle:
            for t in study.trials:
                record = {
                    "number": t.number,
                    "state": str(t.state),
                    "value": t.value,
                    "values": t.values,
                    "params": t.params,
                    "user_attrs": t.user_attrs,
                    "datetime_start": str(t.datetime_start),
                    "datetime_complete": str(t.datetime_complete),
                }
                handle.write(json.dumps(record) + "\n")
    if args.results_csv:
        _write_trials_csv(Path(args.results_csv), study.trials)

    if args.multi_objective:
        best_trials = study.best_trials
        if args.trial_log:
            print(f"Finished {len(study.trials)} trials. Pareto front size: {len(best_trials)}")
            for t in best_trials:
                print(f"trial={t.number} te={t.values[0]:.6f} re={t.values[1]:.6f} params={t.params}")
        if args.pareto_params_yaml:
            pareto_payload = {
                "mode": "fixed",
                "pareto_trials": [
                    {
                        "trial": t.number,
                        "values": t.values,
                        "params": t.params,
                        "metrics": {"te": t.user_attrs.get("te"), "re": t.user_attrs.get("re")},
                    }
                    for t in best_trials
                ],
            }
            write_yaml_config(Path(args.pareto_params_yaml), pareto_payload)
    else:
        if args.trial_log:
            print(f"Finished {len(study.trials)} trials. Best value: {study.best_value:.6f}")
            print(f"Best params: {study.best_params}")
        if args.best_params_yaml:
            best = study.best_trial
            resolved = normalize_fixed_params(best.user_attrs.get("resolved_params", {}))
            if not resolved:
                resolved = normalize_fixed_params(args.fixed_params or {})
                resolved.update(normalize_fixed_params(best.params))
            payload = {
                "mode": "fixed",
                "fixed_params": resolved,
                "best_value": best.value,
                "metrics": {"te": best.user_attrs.get("te"), "re": best.user_attrs.get("re")},
                "study_name": args.study_name,
            }
            write_yaml_config(Path(args.best_params_yaml), payload)


if __name__ == "__main__":
    main()
