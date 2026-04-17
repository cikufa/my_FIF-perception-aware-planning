#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
from pathlib import Path

try:
    import yaml  # type: ignore
except Exception:
    yaml = None


def main():
    root_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Analyze all variations under the trace root."
    )
    default_trace_root = os.environ.get(
        "FOV_TRACE_ROOT",
        str(root_dir / "act_map_exp" / "trace_r2_a20"),
    )
    parser.add_argument(
        "--top-dir",
        default=default_trace_root,
        help="Top directory that contains variation results.",
    )
    parser.add_argument(
        "--dataset",
        choices=["r2_a20", "r1_a30"],
        help="Shortcut for trace root (uses quad_rrt/warehouse/warehouse_all.yaml for --include).",
    )
    parser.add_argument(
        "--warehouse-all-yaml",
        default=None,
        help="Override warehouse yaml for --include views (default: quad_rrt/warehouse/warehouse_all.yaml).",
    )
    parser.add_argument(
        "--base-ana-cfg",
        default=str(root_dir / "act_map_exp" / "params" / "quad_rrt" / "base_analysis_cfg.yaml"),
        help="Base analysis config for analyze_pose_errors.py.",
    )
    parser.add_argument("--plt-min-ratio", type=float, default=None)
    parser.add_argument("--plt-max-ratio", type=float, default=None)
    parser.add_argument(
        "--merge-optimized-from",
        default=None,
        help="Passed to analyze_pose_errors.py: extra tree with optimized_path_yaw (e.g. traj_opt_xyz).",
    )
    parser.add_argument(
        "--analysis-artifacts-dir",
        default=None,
        help="Directory for analysis_outputs/ and summaries (default: same as --top-dir).",
    )
    parser.add_argument(
        "--include-trajs",
        default=None,
        help="Comma-separated config folder names under top_dir (overrides warehouse yaml include list).",
    )

    args = parser.parse_args()
    if args.dataset:
        args.top_dir = str(root_dir / "act_map_exp" / f"trace_{args.dataset}")

    include_views = []
    if args.include_trajs:
        include_views = [v.strip() for v in args.include_trajs.split(",") if v.strip()]
    if args.warehouse_all_yaml:
        warehouse_yaml = Path(args.warehouse_all_yaml).resolve()
    else:
        warehouse_yaml = (
            root_dir
            / "act_map_exp"
            / "params"
            / "quad_rrt"
            / "warehouse"
            / "warehouse_all.yaml"
        )
    if yaml is None:
        raise SystemExit("PyYAML is required to read warehouse_all.yaml.")
    if not include_views and warehouse_yaml.exists():
        data = yaml.safe_load(warehouse_yaml.read_text(encoding="utf-8")) or {}
        base_map = {}
        if isinstance(data, dict):
            grp = data.get("all_cfg") or data.get("all")
            if isinstance(grp, dict):
                base_map = grp.get("base", {}) or {}
        if isinstance(base_map, dict):
            include_views = [v for v in base_map.values() if isinstance(v, str)]

    analyze_script = root_dir / "act_map_exp" / "scripts" / "analyze_pose_errors.py"
    cmd = [
        "python3",
        str(analyze_script),
        str(Path(args.top_dir).resolve()),
        "--base_ana_cfg",
        str(Path(args.base_ana_cfg).resolve()),
        "--multiple",
    ]
    if include_views:
        cmd += ["--include", ",".join(include_views)]
    if args.plt_min_ratio is not None:
        cmd += ["--plt_min_ratio", str(args.plt_min_ratio)]
    if args.plt_max_ratio is not None:
        cmd += ["--plt_max_ratio", str(args.plt_max_ratio)]
    if args.merge_optimized_from:
        cmd += [
            "--merge_optimized_from",
            str(Path(args.merge_optimized_from).resolve()),
        ]
    if args.analysis_artifacts_dir:
        cmd += [
            "--analysis_artifacts_dir",
            str(Path(args.analysis_artifacts_dir).resolve()),
        ]
    subprocess.run(cmd, check=True)
    _relocate_outputs(
        Path(args.analysis_artifacts_dir).resolve()
        if args.analysis_artifacts_dir
        else Path(args.top_dir).resolve()
    )


def _relocate_outputs(top_dir: Path):
    analysis_dir = top_dir / "analysis_outputs"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    base_cfg = top_dir / "base_analysis_cfg.yaml"
    if base_cfg.exists():
        dest = analysis_dir / base_cfg.name
        if dest.exists():
            dest.unlink()
        shutil.move(str(base_cfg), str(dest))

    # Remove duplicate top-level summaries; keep per-mode outputs only.
    for entry in top_dir.iterdir():
        if entry.is_file() and entry.name.startswith("overall_"):
            entry.unlink()
    # Also drop any root-level duplicates inside analysis_outputs.
    for entry in analysis_dir.iterdir():
        if entry.is_file() and entry.name.startswith("overall_"):
            entry.unlink()


if __name__ == "__main__":
    main()
