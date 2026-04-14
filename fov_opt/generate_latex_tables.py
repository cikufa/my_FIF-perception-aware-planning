#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

try:
    import yaml  # type: ignore
except Exception:
    yaml = None


DEFAULT_TYPES = [
    "none",
    "pc_det",
    "pc_trace",
    "gp_det",
    "gp_trace",
    "quad_det",
    "quad_trace",
    "optimized_path_yaw",
]

TYPE_LABELS = {
    "none": "No Info",
    "pc_det": "PC-det",
    "pc_trace": "PC-Trace",
    "gp_det": "GP-det",
    "gp_trace": "GP-Trace",
    "quad_det": "Quad-det",
    "quad_trace": "Quad-Trace",
    "optimized_path_yaw": "optimized(ours)",
}

VIEW_LABELS = {
    "top": "Top",
    "diagonal": "Diagonal",
    "bottom": "Bottom",
    "left": "Left",
    "bigU": "BigU",
    "smallU": "SmallU",
    "mid": "Mid",
}


def _mean(vals):
    vals = [v for v in vals if isinstance(v, (int, float)) and math.isfinite(v)]
    if not vals:
        return float("nan")
    return sum(vals) / len(vals)


def _read_float_file(path: Path):
    if not path.exists():
        return []
    try:
        data = path.read_text().strip().split()
        return [float(x) for x in data if x]
    except Exception:
        return []


def _count_iters(path: Path):
    if not path.exists():
        return float("nan")
    lines = [ln for ln in path.read_text().splitlines() if ln.strip() and not ln.startswith("#")]
    if not lines:
        return float("nan")
    return float(len(lines))


def _load_views(root: Path):
    if yaml is None:
        raise SystemExit("PyYAML is required to read warehouse_all.yaml.")
    cfg = root / "act_map_exp" / "params" / "quad_rrt" / "warehouse" / "warehouse_all.yaml"
    data = yaml.safe_load(cfg.read_text(encoding="utf-8")) or {}
    base_map = data.get("all_cfg", {}).get("base", {}) if isinstance(data, dict) else {}
    return [v for v in base_map.values() if isinstance(v, str)]


def _load_per_cfg_stats(root: Path, dataset: str):
    path = (
        root
        / "act_map_exp"
        / f"trace_{dataset}"
        / "analysis_outputs"
        / "finite"
        / "overall_per_cfg_stats.csv"
    )
    rows = {}
    if not path.exists():
        return rows
    with path.open(encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows[(row.get("cfg"), row.get("type"))] = row
    return rows


def _fmt_fail_cell(row):
    if row is None:
        return "X"
    try:
        total = int(row.get("total", 0))
    except Exception:
        total = 0
    if total <= 0:
        return "X"
    try:
        ratio = float(row.get("nan_ratio", "nan"))
    except Exception:
        ratio = float("nan")
    if not math.isfinite(ratio):
        return "--"
    return f"{int(round(ratio * 100.0))}%"


def _latex_registration_table(root: Path, datasets, views, types):
    header = ["", "cfg."] + [TYPE_LABELS.get(t, t) for t in types]
    lines = []
    lines.append("% Registration failure rates table (booktabs, multirow).")
    lines.append("% Requires: \\usepackage{booktabs} and \\usepackage{multirow}")
    lines.append("\\begin{table*}[t]")
    lines.append("\\centering")
    lines.append("\\caption{Registration failure rates (\\% of failed localizations). "
                 "X denotes no valid solution (no samples).}")
    lines.append("\\begin{tabular}{ll" + "r" * len(types) + "}")
    lines.append("\\toprule")
    lines.append(" & ".join(header) + " \\\\")
    lines.append("\\midrule")

    for dataset in datasets:
        rows = _load_per_cfg_stats(root, dataset)
        ds_label = dataset.replace("_", "-")
        for i, v in enumerate(views):
            cfg_label = VIEW_LABELS.get(v, v.title())
            ds_cell = f"\\multirow{{{len(views)}}}{{*}}{{{ds_label}}}" if i == 0 else ""
            row = [ds_cell, cfg_label]
            for t in types:
                row.append(_fmt_fail_cell(rows.get((v, t))))
            lines.append(" & ".join(row) + " \\\\")
        lines.append("\\midrule")

    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\end{table*}")
    lines.append("")
    lines.append("% Inputs:")
    lines.append("% - analysis_outputs/finite/overall_per_cfg_stats.csv")
    lines.append("% - nan_ratio is used as failure rate; total==0 => X.")
    return "\n".join(lines)


def _latex_planning_table(root: Path, datasets, views, types):
    methods = [(t, TYPE_LABELS.get(t, t)) for t in types]
    out = []
    out.append("% Planning time/iterations table (booktabs).")
    out.append("\\begin{table}[t]")
    out.append("\\centering")
    out.append("\\caption{Average RRT planning iterations and time (s) over all maps. "
               "Optimized(ours) time adds RRT plan time (No Info.) + FoV optimization time.}")
    out.append("\\begin{tabular}{l" + "r" * (2 * len(datasets)) + "}")
    out.append("\\toprule")
    if len(datasets) == 2:
        out.append(" & \\multicolumn{2}{c}{%s} & \\multicolumn{2}{c}{%s} \\\\" % (
            datasets[0].replace("_", "-"), datasets[1].replace("_", "-")
        ))
    else:
        out.append(" & " + " & ".join(d.replace("_", "-") for d in datasets) + " \\\\")
    out.append("Method & " + " & ".join(["iter. & time (s)"] * len(datasets)) + " \\\\")
    out.append("\\midrule")

    for key, label in methods:
        row = [label]
        for dataset in datasets:
            trace = root / "act_map_exp" / f"trace_{dataset}"
            iter_vals = []
            time_vals = []
            for v in views:
                if key == "optimized_path_yaw":
                    iters = _count_iters(trace / v / f"{v}_none" / "rrt_stats.txt")
                    rrt_t = _mean(_read_float_file(trace / v / f"{v}_none" / "rrt_plan_time_sec.txt"))
                    opt_t = _mean(
                        _read_float_file(
                            trace / v / f"{v}_none" / "optimized_path_yaw" / "optimization_time_sec.txt"
                        )
                    )
                    if math.isfinite(iters):
                        iter_vals.append(iters)
                    if math.isfinite(rrt_t) and math.isfinite(opt_t):
                        time_vals.append(rrt_t + opt_t)
                else:
                    iters = _count_iters(trace / v / f"{v}_{key}" / "rrt_stats.txt")
                    rrt_t = _mean(_read_float_file(trace / v / f"{v}_{key}" / "rrt_plan_time_sec.txt"))
                    if math.isfinite(iters):
                        iter_vals.append(iters)
                    if math.isfinite(rrt_t):
                        time_vals.append(rrt_t)
            iter_avg = _mean(iter_vals)
            time_avg = _mean(time_vals)
            iter_str = "--" if not math.isfinite(iter_avg) else f"{iter_avg:.1f}"
            time_str = "--" if not math.isfinite(time_avg) else f"{time_avg:.2f}"
            row.extend([iter_str, time_str])
        out.append(" & ".join(row) + " \\\\")

    out.append("\\bottomrule")
    out.append("\\end{tabular}")
    out.append("\\end{table}")
    out.append("")
    out.append("% Inputs:")
    out.append("% - iter: count of rows in rrt_stats.txt (per map, per method)")
    out.append("% - time: mean of rrt_plan_time_sec.txt (per map, per method)")
    out.append("% - optimized(ours) time: rrt_plan_time_sec.txt (none) + optimization_time_sec.txt (optimized_path_yaw)")
    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser(
        description="Generate LaTeX tables for registration failures and planning time."
    )
    parser.add_argument(
        "--root",
        default=None,
        help="Repo root (defaults to parent of this script).",
    )
    parser.add_argument(
        "--datasets",
        default="r2_a20,r1_a30",
        help="Comma-separated datasets to include.",
    )
    parser.add_argument(
        "--views",
        default="",
        help="Comma-separated view names to include (default: from warehouse_all.yaml).",
    )
    parser.add_argument(
        "--types",
        default="",
        help="Comma-separated variation types (default: standard set).",
    )
    parser.add_argument(
        "--only",
        choices=["registration", "planning", "both"],
        default="both",
        help="Which table(s) to generate.",
    )
    args = parser.parse_args()

    root = Path(args.root).resolve() if args.root else Path(__file__).resolve().parents[1]
    datasets = [d.strip() for d in args.datasets.split(",") if d.strip()]
    if args.views:
        views = [v.strip() for v in args.views.split(",") if v.strip()]
    else:
        views = _load_views(root)
    types = [t.strip() for t in args.types.split(",") if t.strip()] if args.types else list(DEFAULT_TYPES)

    out = []
    if args.only in ("registration", "both"):
        out.append(_latex_registration_table(root, datasets, views, types))
    if args.only == "both":
        out.append("")
    if args.only in ("planning", "both"):
        out.append(_latex_planning_table(root, datasets, views, types))
    print("\n".join(out))


if __name__ == "__main__":
    main()
