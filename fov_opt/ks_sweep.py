#!/usr/bin/env python3
import argparse
import json
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path

try:
    import yaml  # type: ignore
except Exception:
    print("PyYAML required.")
    raise

ROOT = Path(__file__).resolve().parents[1]


def load_yaml(path: Path):
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return data if isinstance(data, dict) else {}


def compute_base_ks(alpha_deg: float, transition_deg: float):
    if alpha_deg <= 0 or transition_deg <= 0:
        return None
    alpha_rad = math.radians(alpha_deg)
    delta_rad = math.radians(transition_deg)
    sin_alpha = math.sin(alpha_rad)
    if abs(sin_alpha) < 1e-6 or delta_rad <= 0:
        return None
    return 2.94 / (delta_rad * sin_alpha)


def parse_schedule_last_alpha(schedule):
    if schedule is None:
        return None
    if isinstance(schedule, (list, tuple)):
        vals = schedule
    else:
        vals = [p.strip() for p in str(schedule).split(",") if p.strip()]
    try:
        return float(vals[-1]) if vals else None
    except Exception:
        return None


def linspace(lo, hi, n):
    if n <= 1:
        return [0.5 * (lo + hi)]
    step = (hi - lo) / float(n - 1)
    return [lo + i * step for i in range(n)]


def run_fixed(config_path: Path, out_dir: Path, along_path: bool):
    cmd = [
        sys.executable,
        str(ROOT / "fov_opt" / "optuna_fov_tune.py"),
        "--config",
        str(config_path),
        "--mode",
        "fixed",
    ]
    if along_path:
        cmd.append("--along-path")
    subprocess.run(cmd, check=True, cwd=str(ROOT / "act_map_exp" / "scripts"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/tmp/optuna_fov_tune_both.yaml")
    parser.add_argument("--best-params", default=str(ROOT / "fov_opt" / "optuna" / "optuna_best_params_both.yaml"))
    parser.add_argument("--out-dir", default="")
    parser.add_argument("--n", type=int, default=5)
    parser.add_argument("--ks-range", type=float, nargs=2)
    parser.add_argument("--ks-transition-deg", type=float,
                        help="Override ks_transition_deg in config (for ks_from_visibility mapping).")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--random", action="store_true")
    parser.add_argument("--along-path", action="store_true", default=True)
    parser.add_argument("--show-subprocess", action="store_true",
                        help="show subprocess output from run_planner_exp")
    args = parser.parse_args()

    base_cfg = Path(args.config).resolve()
    if not base_cfg.exists():
        raise SystemExit(f"Config not found: {base_cfg}")
    best_path = Path(args.best_params).resolve()
    if not best_path.exists():
        raise SystemExit(f"Best params not found: {best_path}")

    cfg = load_yaml(base_cfg)
    best = load_yaml(best_path)
    fixed = best.get("fixed_params") or {}
    if not isinstance(fixed, dict) or not fixed:
        raise SystemExit("No fixed_params in best params yaml.")

    # Determine ks range
    ks_lo = ks_hi = None
    if args.ks_range:
        ks_lo, ks_hi = args.ks_range
    else:
        schedule = fixed.get("fov_schedule", cfg.get("fov_schedule"))
        alpha = parse_schedule_last_alpha(schedule)
        transition_deg = args.ks_transition_deg
        if transition_deg is None:
            transition_deg = fixed.get("ks_transition_deg", cfg.get("ks_transition_deg", 0))
        base_ks = compute_base_ks(alpha or 0, transition_deg or 0)
        mults = cfg.get("ks_visibility_range_multipliers", [0.8, 1.2])
        if base_ks is None:
            raise SystemExit("Could not derive ks range; provide --ks-range.")
        ks_lo = base_ks * float(mults[0])
        ks_hi = base_ks * float(mults[1])

    if ks_lo is None or ks_hi is None:
        raise SystemExit("Invalid ks range.")

    if not args.out_dir:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_dir = ROOT / "fov_opt" / "debug" / f"ks_sweep_{stamp}"
    else:
        out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    random.seed(args.seed)
    if args.random:
        ks_samples = [random.uniform(ks_lo, ks_hi) for _ in range(args.n)]
    else:
        ks_samples = linspace(ks_lo, ks_hi, args.n)

    variations = []
    variations.append({
        "name": "ks_from_visibility",
        "ks_from_visibility": True,
        "ks": fixed.get("ks"),
    })
    for idx, ks in enumerate(ks_samples, start=1):
        variations.append({
            "name": f"ks_{idx:02d}",
            "ks_from_visibility": False,
            "ks": ks,
        })

    summary = []
    for var in variations:
        var_dir = out_dir / var["name"]
        var_dir.mkdir(parents=True, exist_ok=True)
        # build config
        cfg_i = dict(cfg)
        cfg_i["mode"] = "fixed"
        cfg_i["fixed_params"] = dict(fixed)
        if args.ks_transition_deg is not None:
            cfg_i["ks_transition_deg"] = float(args.ks_transition_deg)
        if var["ks"] is not None:
            cfg_i["fixed_params"]["ks"] = float(var["ks"])
        cfg_i["ks_from_visibility"] = bool(var["ks_from_visibility"])

        if args.show_subprocess:
            cfg_i["quiet_subprocess"] = False

            def _append_show(cmd):
                if not cmd:
                    return cmd
                parts = [p.strip() for p in cmd.split("&&")]
                updated = []
                for part in parts:
                    if "run_planner_exp.py" in part and "--show_subprocess" not in part:
                        part = part + " --show_subprocess --verbose"
                    updated.append(part)
                return " && ".join(updated)

            cfg_i["run_cmd_registration_along_path"] = _append_show(
                cfg_i.get("run_cmd_registration_along_path")
            )
            cfg_i["run_cmd_registration_normal"] = _append_show(
                cfg_i.get("run_cmd_registration_normal")
            )

        cfg_i["results_jsonl"] = str(var_dir / "results.jsonl")
        cfg_i["results_csv"] = str(var_dir / "results.csv")
        cfg_i["best_params_yaml"] = str(var_dir / "best_params.yaml")
        cfg_i["pareto_params_yaml"] = ""

        config_out = var_dir / "config.yaml"
        config_out.write_text(yaml.safe_dump(cfg_i, sort_keys=False), encoding="utf-8")

        print(f"=== Running {var['name']} (ks_from_visibility={var['ks_from_visibility']}, ks={var['ks']}) ===")
        run_fixed(config_out, var_dir, args.along_path)

        # collect metrics
        metrics = {}
        best_data = load_yaml(var_dir / "best_params.yaml")
        if isinstance(best_data, dict):
            metrics = best_data.get("metrics", {}) or {}
        summary.append({
            "name": var["name"],
            "ks_from_visibility": var["ks_from_visibility"],
            "ks": var["ks"],
            "te": metrics.get("te"),
            "re": metrics.get("re"),
            "best_value": best_data.get("best_value") if isinstance(best_data, dict) else None,
        })

    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\nSummary saved: {summary_path}")
    for row in summary:
        print(row)


if __name__ == "__main__":
    main()
