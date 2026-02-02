#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path


def main():
    root_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Analyze all variations under act_map_exp/trace."
    )
    parser.add_argument(
        "--top-dir",
        default=str(root_dir / "act_map_exp" / "trace"),
        help="Top directory that contains variation results.",
    )
    parser.add_argument(
        "--base-ana-cfg",
        default=str(root_dir / "act_map_exp" / "params" / "quad_rrt" / "base_analysis_cfg.yaml"),
        help="Base analysis config for analyze_pose_errors.py.",
    )
    parser.add_argument("--plt-min-ratio", type=float, default=None)
    parser.add_argument("--plt-max-ratio", type=float, default=None)

    args = parser.parse_args()

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
