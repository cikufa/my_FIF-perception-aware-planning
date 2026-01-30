#!/usr/bin/env python3
import argparse
import json
import os
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
    if "visibility_angle_deg" in fixed:
        params["visibility_angle_deg"] = fixed["visibility_angle_deg"]
    elif args.tune_visibility:
        params["visibility_angle_deg"] = trial.suggest_float(
            "visibility_angle_deg",
            args.visibility_range[0],
            args.visibility_range[1],
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
    if "min_step_deg" in fixed:
        params["min_step_deg"] = fixed["min_step_deg"]
    elif args.tune_step_limits:
        params["min_step_deg"] = trial.suggest_float(
            "min_step_deg", args.min_step_range[0], args.min_step_range[1]
        )
    if "max_step_deg" in fixed:
        params["max_step_deg"] = fixed["max_step_deg"]
    elif args.tune_step_limits:
        params["max_step_deg"] = trial.suggest_float(
            "max_step_deg", args.max_step_range[0], args.max_step_range[1]
        )
    if "traj_jac_step" in fixed:
        params["traj_jac_step"] = fixed["traj_jac_step"]
    elif args.tune_traj_jac_step:
        params["traj_jac_step"] = trial.suggest_float(
            "traj_jac_step",
            args.traj_jac_step_range[0],
            args.traj_jac_step_range[1],
        )
    if "fov_schedule" in fixed:
        params["fov_schedule"] = fixed["fov_schedule"]
    elif args.tune_fov_schedule:
        params["fov_schedule"] = trial.suggest_categorical(
            "fov_schedule", args.fov_schedule_options
        )
    if args.ks_from_visibility and "ks" not in fixed:
        alpha = None
        if "visibility_angle_deg" in params:
            alpha = params["visibility_angle_deg"]
        elif "fov_schedule" in params:
            alpha = parse_schedule_last_alpha(params["fov_schedule"])
        if alpha is not None and args.ks_transition_deg:
            ks_suggested = suggest_ks(alpha, args.ks_transition_deg)
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
        if alpha is not None:
            params["visibility_alpha_deg"] = alpha
    return params


def make_env(params, base_env):
    env = dict(base_env)
    if "max_iter" in params:
        env["FOV_OPT_MAX_ITER"] = str(params["max_iter"])
    if "ks" in params:
        env["FOV_OPT_KS"] = f"{params['ks']:.8f}"
    if "visibility_angle_deg" in params:
        env["FOV_OPT_VIS_ANGLE_DEG"] = f"{params['visibility_angle_deg']:.8f}"
    if "base_step_scale" in params:
        env["FOV_OPT_BASE_STEP_SCALE"] = f"{params['base_step_scale']:.8f}"
    if "min_step_deg" in params:
        env["FOV_OPT_MIN_STEP_DEG"] = f"{params['min_step_deg']:.8f}"
    if "max_step_deg" in params:
        env["FOV_OPT_MAX_STEP_DEG"] = f"{params['max_step_deg']:.8f}"
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
    if args.trial_log:
        print("[trial {}] optimization start".format(trial_id))
    run_trial_command(args.run_cmd_optimization, workdir, env, args.quiet_subprocess)
    if args.trial_log:
        print("[trial {}] registration start".format(trial_id))
    run_trial_command(reg_cmd, workdir, env, args.quiet_subprocess)


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
        "tune_visibility",
        "tune_base_step",
        "tune_step_limits",
        "tune_traj_jac_step",
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
        supported = {
            "max_iter",
            "ks",
            "visibility_angle_deg",
            "base_step_scale",
            "min_step_deg",
            "max_step_deg",
            "traj_jac_step",
            "fov_schedule",
        }
        if not isinstance(fixed, dict) or not any(k in fixed for k in supported):
            missing.append("fixed_params (with at least one supported key)")

    if args.tune_max_iter and "max_iter" not in fixed:
        require("max_iter_range")
    if args.tune_visibility and "visibility_angle_deg" not in fixed:
        require("visibility_range")
    if args.tune_base_step and "base_step_scale" not in fixed:
        require("base_step_scale_range")
        require("base_step_scale_log")
    if args.tune_step_limits:
        if "min_step_deg" not in fixed:
            require("min_step_range")
        if "max_step_deg" not in fixed:
            require("max_step_range")
    if args.tune_traj_jac_step and "traj_jac_step" not in fixed:
        require("traj_jac_step_range")
    if args.tune_fov_schedule and "fov_schedule" not in fixed:
        require("fov_schedule_options")

    if args.tune_ks and "ks" not in fixed:
        require("ks_log")
        if not args.ks_from_visibility:
            require("ks_range")
    if args.ks_from_visibility:
        require("ks_transition_deg")
        require("ks_visibility_range_multipliers")
        has_alpha = False
        if args.tune_visibility and "visibility_angle_deg" not in fixed:
            has_alpha = True
        if args.tune_fov_schedule and "fov_schedule" not in fixed:
            has_alpha = True
        if "visibility_angle_deg" in fixed or "fov_schedule" in fixed:
            has_alpha = True
        if not has_alpha:
            missing.append("visibility_angle_deg or fov_schedule (for ks_from_visibility)")

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

    if args.fixed_params is not None and not isinstance(args.fixed_params, dict):
        raise SystemExit("fixed_params must be a mapping of param names to values.")


def main():
    parser = argparse.ArgumentParser(description="Optuna tuning for FoV optimization.")
    parser.add_argument("--config", default="", help="Path to YAML config file.")
    parser.add_argument("--mode", choices=["tune", "fixed"], default=None)
    parser.add_argument("--along-path", action="store_true", default=None)
    parser.add_argument("--run-cmd", help="Command that runs the full pipeline.")
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
    parser.add_argument("--tune-visibility", action="store_true", default=None)
    parser.add_argument("--visibility-range", type=float, nargs=2)
    parser.add_argument("--tune-base-step", action="store_true", default=None)
    parser.add_argument("--base-step-scale-range", type=float, nargs=2)
    parser.add_argument("--base-step-scale-log", action="store_true", default=None)
    parser.add_argument("--tune-step-limits", action="store_true", default=None)
    parser.add_argument("--min-step-range", type=float, nargs=2)
    parser.add_argument("--max-step-range", type=float, nargs=2)
    parser.add_argument("--tune-traj-jac-step", action="store_true", default=None)
    parser.add_argument("--traj-jac-step-range", type=float, nargs=2)
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

    if args.mode == "fixed":
        args.tune_max_iter = False
        args.tune_ks = False
        args.tune_visibility = False
        args.tune_base_step = False
        args.tune_step_limits = False
        args.tune_traj_jac_step = False
        args.tune_fov_schedule = False

    validate_config(args)

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

    workdir = Path(args.workdir).resolve()
    error_stat_path = Path(args.error_stat).resolve() if args.error_stat else None
    pose_errors_path = Path(args.pose_errors).resolve() if args.pose_errors else None
    trial_root = Path(args.trial_dir_root).resolve() if args.trial_dir_root else None

    def objective(trial):
        params = build_search_space(trial, args)
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

        start = time.time()
        if args.trial_log:
            print("[trial {}] start".format(trial.number))
        run_trial_pipeline(trial.number, args, env, workdir)
        elapsed = time.time() - start

        if args.error_stat_files:
            te, re = parse_error_stat_files(args.error_stat_files, args.stat)
        elif args.pose_errors_files:
            te, re = parse_pose_errors_files(args.pose_errors_files, args.stat)
        elif error_stat_path and error_stat_path.exists():
            te, re = parse_error_stat(error_stat_path, args.stat)
        elif pose_errors_path and pose_errors_path.exists():
            te, re = parse_pose_errors(pose_errors_path, args.stat)
        else:
            raise RuntimeError("No error file found after run.")

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
            if vis_alpha is None:
                vis_alpha = params.get("visibility_angle_deg")
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
                    "visibility_angle_deg",
                    "visibility_alpha_deg",
                    "base_step_scale",
                    "min_step_deg",
                    "max_step_deg",
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
            payload = {
                "mode": "fixed",
                "fixed_params": best.params,
                "best_value": best.value,
                "metrics": {"te": best.user_attrs.get("te"), "re": best.user_attrs.get("re")},
                "study_name": args.study_name,
            }
            write_yaml_config(Path(args.best_params_yaml), payload)


if __name__ == "__main__":
    main()
