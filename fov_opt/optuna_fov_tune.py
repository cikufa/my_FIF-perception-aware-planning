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

import optuna


STAT_INDEX = {
    "min": 0,
    "max": 1,
    "avg": 2,
    "std": 3,
}

SUPPORTED_FIXED_PARAMS = {
    "max_iter",
    "ks",
    "ks_transition_deg",
    "base_step_scale",
    "min_step_deg",
    "max_step_deg",
    "step_norm_mode",
    "traj_jac_step",
    "fov_schedule",
}


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


def parse_pose_errors(path: Path, stat: str):
    te_vals = []
    re_vals = []
    total = 0
    with path.open("r", encoding="utf-8") as handle:
        for line_idx, line in enumerate(handle):
            if line_idx == 0:
                continue
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            total += 1
            if parts[1] != "nan":
                te_vals.append(float(parts[1]))
            if parts[2] != "nan":
                re_vals.append(float(parts[2]))
    if not te_vals or not re_vals:
        raise RuntimeError(f"pose_errors file has no usable entries: {path}")
    return compute_stat(te_vals, stat), compute_stat(re_vals, stat)


def parse_pose_errors_files(paths, stat: str):
    te_vals = []
    re_vals = []
    for path in paths:
        path_obj = Path(path)
        if not path_obj.exists():
            raise RuntimeError(f"pose_errors file missing: {path_obj}")
        with path_obj.open("r", encoding="utf-8") as handle:
            for line_idx, line in enumerate(handle):
                if line_idx == 0:
                    continue
                parts = line.strip().split()
                if len(parts) < 3:
                    continue
                if parts[1] != "nan":
                    te_vals.append(float(parts[1]))
                if parts[2] != "nan":
                    re_vals.append(float(parts[2]))
    if not te_vals or not re_vals:
        raise RuntimeError("pose_errors files have no usable entries.")
    return compute_stat(te_vals, stat), compute_stat(re_vals, stat)


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


def build_search_space(trial, args):
    params = {}
    fixed = args.fixed_params or {}
    if "max_iter" in fixed:
        params["max_iter"] = fixed["max_iter"]
    elif args.tune_max_iter:
        params["max_iter"] = trial.suggest_int(
            "max_iter", args.max_iter_range[0], args.max_iter_range[1]
        )
    if "ks" in fixed:
        params["ks"] = fixed["ks"]
    elif args.tune_ks and not args.ks_from_visibility:
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
    if "traj_jac_step" in fixed:
        params["traj_jac_step"] = fixed["traj_jac_step"]
    elif args.tune_traj_jac_step:
        params["traj_jac_step"] = trial.suggest_float(
            "traj_jac_step",
            args.traj_jac_step_range[0],
            args.traj_jac_step_range[1],
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
    if args.ks_from_visibility and "ks" not in fixed:
        alpha = None
        if "fov_schedule" in params:
            alpha = parse_schedule_last_alpha(params["fov_schedule"])
        ks_transition_deg = params.get("ks_transition_deg", args.ks_transition_deg)
        if alpha is not None and ks_transition_deg:
            ks_suggested = suggest_ks(alpha, ks_transition_deg)
            if ks_suggested is not None:
                if args.tune_ks:
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


def make_env(params, base_env):
    env = dict(base_env)
    if "max_iter" in params:
        env["FOV_OPT_MAX_ITER"] = str(params["max_iter"])
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
    if "traj_jac_step" in params:
        env["FOV_OPT_TRAJ_JAC_STEP"] = f"{params['traj_jac_step']:.8f}"
    if "fov_schedule" in params:
        env["FOV_OPT_FOV_SCHEDULE"] = params["fov_schedule"]
    if args_warm_start_enabled := env.get("_FOV_OPT_WARM_START"):
        env["FOV_OPT_WARM_START"] = args_warm_start_enabled
    if warm_start_file := env.get("_FOV_OPT_WARM_START_FILE"):
        env["FOV_OPT_WARM_START_FILE"] = warm_start_file
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
    if "--max_reg_images" in cmd:
        return cmd
    return "{} --max_reg_images {}".format(cmd, max_imgs)


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
                opt_dir = Path(trace_root) / name / f"{name}_none" / "optimized"
                opt_specs.append(f"{name}={opt_dir}")
            tokens += ["--optimized_dirs"] + opt_specs
    return " ".join(shlex.quote(tok) for tok in tokens)


def run_trial_pipeline(trial_id, args, env, workdir):
    has_split_cmd = bool(
        args.run_cmd_optimization
        or args.run_cmd_registration_along_path
        or args.run_cmd_registration_normal
    )
    if not has_split_cmd:
        if args.trial_log:
            print("[trial {}] run pipeline".format(trial_id))
        run_trial_command(args.run_cmd, workdir, env, args.quiet_subprocess)
        return

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
    run_trial_command(args.run_cmd_optimization, workdir, env, args.quiet_subprocess)
    if args.trial_log:
        max_imgs = int(getattr(args, "max_reg_images_per_trial", 0) or 0)
        if max_imgs > 0:
            print("[trial {}] registration start (max_reg_images={})".format(
                trial_id, max_imgs))
        else:
            print("[trial {}] registration start".format(trial_id))
    run_trial_command(reg_cmd, workdir, env, args.quiet_subprocess)


def load_metrics(args, error_stat_path, pose_errors_path,
                 error_stat_files=None, pose_errors_files=None):
    if error_stat_files is None:
        error_stat_files = args.error_stat_files
    if pose_errors_files is None:
        pose_errors_files = args.pose_errors_files
    if error_stat_files:
        return parse_error_stat_files(error_stat_files, args.stat)
    if pose_errors_files:
        return parse_pose_errors_files(pose_errors_files, args.stat)
    if error_stat_path and error_stat_path.exists():
        return parse_error_stat(error_stat_path, args.stat)
    if pose_errors_path and pose_errors_path.exists():
        return parse_pose_errors(pose_errors_path, args.stat)
    raise RuntimeError("No error file found after run.")


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
        rows.append(row)
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = param_keys + ["te", "re"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def normalize_fixed_params(raw):
    if not isinstance(raw, dict):
        return {}
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
        "tune_max_iter",
        "tune_ks",
        "tune_ks_transition_deg",
        "tune_base_step",
        "tune_min_step_deg",
        "tune_max_step_deg",
        "tune_traj_jac_step",
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

    if args.tune_max_iter and "max_iter" not in fixed:
        require("max_iter_range")
    if args.tune_base_step and "base_step_scale" not in fixed:
        require("base_step_scale_range")
        require("base_step_scale_log")
    if args.tune_min_step_deg and "min_step_deg" not in fixed:
        require("min_step_deg_range")
    if args.tune_max_step_deg and "max_step_deg" not in fixed:
        require("max_step_deg_range")
    if args.tune_traj_jac_step and "traj_jac_step" not in fixed:
        require("traj_jac_step_range")
    if args.tune_step_norm_mode and "step_norm_mode" not in fixed:
        require("step_norm_mode_options")
    if args.tune_fov_schedule and "fov_schedule" not in fixed:
        require("fov_schedule_options")

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


def _tunable_keys(args, fixed):
    keys = []
    if args.tune_max_iter and "max_iter" not in fixed:
        keys.append("max_iter")
    if args.tune_ks and "ks" not in fixed:
        keys.append("ks")
    if args.tune_ks_transition_deg and "ks_transition_deg" not in fixed:
        keys.append("ks_transition_deg")
    if args.tune_base_step and "base_step_scale" not in fixed:
        keys.append("base_step_scale")
    if args.tune_min_step_deg and "min_step_deg" not in fixed:
        keys.append("min_step_deg")
    if args.tune_max_step_deg and "max_step_deg" not in fixed:
        keys.append("max_step_deg")
    if args.tune_traj_jac_step and "traj_jac_step" not in fixed:
        keys.append("traj_jac_step")
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
    max_iter_range = getattr(args, "max_iter_range", None)
    ks_range = getattr(args, "ks_range", None)
    ks_transition_deg_range = getattr(args, "ks_transition_deg_range", None)
    base_step_scale_range = getattr(args, "base_step_scale_range", None)
    min_step_deg_range = getattr(args, "min_step_deg_range", None)
    max_step_deg_range = getattr(args, "max_step_deg_range", None)
    traj_jac_step_range = getattr(args, "traj_jac_step_range", None)
    step_norm_mode_options = getattr(args, "step_norm_mode_options", None)
    fov_schedule_options = getattr(args, "fov_schedule_options", None)

    if "max_iter" in initial and max_iter_range:
        in_range("max_iter", initial["max_iter"], max_iter_range[0], max_iter_range[1])
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
    if "traj_jac_step" in initial and traj_jac_step_range:
        in_range(
            "traj_jac_step",
            initial["traj_jac_step"],
            traj_jac_step_range[0],
            traj_jac_step_range[1],
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
    parser.add_argument("--trial-dir-root", help="Optional base dir for trial outputs.")
    parser.add_argument("--log-jacobian", type=int, choices=[0, 1])

    parser.add_argument("--tune-max-iter", action="store_true", default=None)
    parser.add_argument("--max-iter-range", type=int, nargs=2)
    parser.add_argument("--tune-ks", action="store_true", default=None)
    parser.add_argument("--ks-range", type=float, nargs=2)
    parser.add_argument("--ks-log", action="store_true", default=None)
    parser.add_argument("--tune-ks-transition-deg", action="store_true", default=None)
    parser.add_argument("--ks-transition-deg-range", type=float, nargs=2)
    parser.add_argument("--tune-base-step", action="store_true", default=None)
    parser.add_argument("--base-step-scale-range", type=float, nargs=2)
    parser.add_argument("--base-step-scale-log", action="store_true", default=None)
    parser.add_argument("--tune-min-step-deg", action="store_true", default=None)
    parser.add_argument("--min-step-deg-range", type=float, nargs=2)
    parser.add_argument("--tune-max-step-deg", action="store_true", default=None)
    parser.add_argument("--max-step-deg-range", type=float, nargs=2)
    parser.add_argument("--tune-traj-jac-step", action="store_true", default=None)
    parser.add_argument("--traj-jac-step-range", type=float, nargs=2)
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

    args = argparse.Namespace(**merged)

    if args.along_path is None:
        args.along_path = False

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
        args.tune_max_iter = False
        args.tune_ks = False
        args.tune_ks_transition_deg = False
        args.tune_base_step = False
        args.tune_min_step_deg = False
        args.tune_max_step_deg = False
        args.tune_traj_jac_step = False
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
        env = make_env(params, base_env)
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
        run_trial_pipeline("fixed", args, env, workdir)
        trace_root = env.get("FOV_TRACE_ROOT")
        error_stat_files = _rewrite_trace_paths(args.error_stat_files, trace_root)
        pose_errors_files = _rewrite_trace_paths(args.pose_errors_files, trace_root)
        if error_stat_path and trace_root:
            error_stat_path = Path(_rewrite_trace_paths([str(error_stat_path)], trace_root)[0])
        if pose_errors_path and trace_root:
            pose_errors_path = Path(_rewrite_trace_paths([str(pose_errors_path)], trace_root)[0])
        te, re = load_metrics(
            args,
            error_stat_path,
            pose_errors_path,
            error_stat_files=error_stat_files,
            pose_errors_files=pose_errors_files,
        )
        elapsed = time.time() - start
        objective = te if args.multi_objective else (args.te_weight * te + args.re_weight * re)

        if args.trial_log:
            print("[fixed] obj={} te={} re={} elapsed={:.2f}s".format(
                "{:.6f}".format(objective) if not args.multi_objective else "(multi)",
                "{:.6f}".format(te),
                "{:.6f}".format(re),
                elapsed,
            ))
            print("[fixed] params {}".format(params))

        if args.results_jsonl:
            results_path = Path(args.results_jsonl)
            record = {
                "mode": "fixed",
                "state": "complete",
                "value": objective if not args.multi_objective else None,
                "values": [te, re] if args.multi_objective else None,
                "params": params,
                "user_attrs": {"te": te, "re": re, "elapsed_sec": elapsed},
            }
            with results_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record) + "\n")
        if args.best_params_yaml:
            payload = {
                "mode": "fixed",
                "fixed_params": normalize_fixed_params(params),
                "best_value": objective if not args.multi_objective else None,
                "metrics": {"te": te, "re": re},
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
        env = make_env(params, base_env)
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
        run_trial_pipeline(trial.number, args, env, workdir)
        elapsed = time.time() - start
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

        te, re = load_metrics(
            args,
            error_stat_path_local,
            pose_errors_path_local,
            error_stat_files=error_stat_files,
            pose_errors_files=pose_errors_files,
        )

        trial.set_user_attr("te", te)
        trial.set_user_attr("re", re)
        trial.set_user_attr("elapsed_sec", elapsed)
        if trial_dir:
            trial.set_user_attr("trial_dir", str(trial_dir))

        trial.set_user_attr("objective", te if args.multi_objective else (args.te_weight * te + args.re_weight * re))
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
            # Print key params in a compact one-line dict.
            compact = {
                k: params.get(k)
                for k in (
                    "max_iter",
                    "ks",
                    "ks_transition_deg",
                    "base_step_scale",
                    "min_step_deg",
                    "max_step_deg",
                    "step_norm_mode",
                    "traj_jac_step",
                    "fov_schedule",
                )
                if k in params
            }
            print("[trial {}] params {}".format(trial.number, compact))
        if args.multi_objective:
            return te, re
        return args.te_weight * te + args.re_weight * re

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
