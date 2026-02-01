#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path


def load_yaml(path: Path):
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "PyYAML is required. Install with: pip install pyyaml"
        ) from exc
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"YAML config must be a mapping: {path}")
    return data


def write_yaml(path: Path, data):
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "PyYAML is required. Install with: pip install pyyaml"
        ) from exc
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False)


def main():
    root_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Apply Optuna best params to a fixed optimization config and run analysis."
    )
    parser.add_argument(
        "--best-yaml",
        default=str(root_dir / "act_map_exp" / "trace" / "optuna_best_params.yaml"),
        help="Optuna best-params YAML.",
    )
    parser.add_argument(
        "--tune-config",
        default=str(root_dir / "fov_opt" / "optuna_fov_tune.yaml"),
        help="Base tuning config to copy pipeline settings from.",
    )
    parser.add_argument(
        "--out-config",
        default=str(root_dir / "fov_opt" / "optuna_fov_fixed.yaml"),
        help="Output fixed-params config YAML.",
    )
    parser.add_argument(
        "--skip-opt",
        action="store_true",
        default=False,
        help="Skip re-running optimization/registration with best params.",
    )
    parser.add_argument(
        "--top-dir",
        default=str(root_dir / "act_map_exp" / "trace"),
        help="Top directory that contains all variation results.",
    )
    parser.add_argument(
        "--base-ana-cfg",
        default=str(root_dir / "act_map_exp" / "params" / "quad_rrt" / "base_analysis_cfg.yaml"),
        help="Base analysis config for analyze_pose_errors.py.",
    )
    parser.add_argument("--plt-min-ratio", type=float, default=None)
    parser.add_argument("--plt-max-ratio", type=float, default=None)
    parser.add_argument("--skip-analyze", action="store_true", default=False)

    args = parser.parse_args()

    best_path = Path(args.best_yaml).resolve()
    base_cfg_path = Path(args.tune_config).resolve()
    out_cfg_path = Path(args.out_config).resolve()

    best_data = load_yaml(best_path)
    fixed_params = best_data.get("fixed_params")
    if not isinstance(fixed_params, dict) or not fixed_params:
        raise SystemExit(f"No fixed_params found in {best_path}")

    base_cfg = load_yaml(base_cfg_path)
    base_cfg["mode"] = "fixed"
    base_cfg["n_trials"] = 1
    base_cfg["fixed_params"] = fixed_params
    write_yaml(out_cfg_path, base_cfg)

    if not args.skip_opt:
        tune_script = root_dir / "fov_opt" / "optuna_fov_tune.py"
        subprocess.run(
            [
                "python3",
                str(tune_script),
                "--config",
                str(out_cfg_path),
                "--mode",
                "fixed",
            ],
            check=True,
        )

    if args.skip_analyze:
        return

    analyze_script = root_dir / "act_map_exp" / "scripts" / "analyze_pose_errors.py"
    cmd = [
        "python3",
        str(analyze_script),
        str(Path(args.top_dir).resolve()),
        "--base_ana_cfg",
        str(Path(args.base_ana_cfg).resolve()),
        "--multiple",
    ]
    if args.plt_min_ratio is not None:
        cmd += ["--plt_min_ratio", str(args.plt_min_ratio)]
    if args.plt_max_ratio is not None:
        cmd += ["--plt_max_ratio", str(args.plt_max_ratio)]
    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
