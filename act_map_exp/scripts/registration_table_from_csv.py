#!/usr/bin/env python3
"""
Build LaTeX rows (or a full table) for registration failure rates from per-trajectory
registration_stats.csv files (as written by analyze_pose_errors.py).

Expected layout:
  <base>/<map_dir>/<trajectory>/registration_stats.csv
  <base>/<map_dir>/<trajectory>/analysis_cfg.yaml  (optional; maps row names to types)
  optimized(ours): <base>/<map_dir>/<traj>/<traj>_none/optimized_path_yaw/pose_errors.txt
    (or pose_errors_path_yaw.txt if present — same preference as analyze_pose_errors.py)

CSV columns: name,non_registered_ratio,registered_ratio,n_failed,n_total
Table columns: none, pc_det, pc_trace, gp_det, gp_trace, quad_det, quad_trace, optimized_path_yaw

The last column uses rotation-error NaNs after the same max_trans / max_rot clipping as
analyze_pose_errors.py (thresholds from that trajectory's analysis_cfg.yaml).

Usage:
  ./registration_table_from_csv.py
  ./registration_table_from_csv.py --base /path/to/act_map_exp --full-table
"""

import argparse
import csv
import math
import os
import sys

try:
    import yaml
except ImportError:
    yaml = None

POSE_E_NM = "pose_errors.txt"
POSE_E_PATH_YAW_NM = "pose_errors_path_yaw.txt"


def _load_pose_error(err_fn, max_trans_e_m=float("nan"), max_rot_e_deg=float("nan")):
    """Match analyze_pose_errors._loadPoseError (rotation list used for registration failure)."""
    trans_e_m = []
    rot_e_deg = []
    with open(err_fn) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            elems = line.strip().split(" ")
            te_i = float(elems[1])
            if not math.isnan(max_trans_e_m) and te_i > max_trans_e_m:
                trans_e_m.append(float("nan"))
            else:
                trans_e_m.append(te_i)
            re_i = float(elems[2])
            if not math.isnan(max_rot_e_deg) and re_i > max_rot_e_deg:
                rot_e_deg.append(float("nan"))
            else:
                rot_e_deg.append(re_i)
    return trans_e_m, rot_e_deg


def _cfg_float(cfg, key):
    if not cfg:
        return float("nan")
    v = cfg.get(key)
    if v is None:
        return float("nan")
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def _analysis_thresholds(yaml_path):
    if yaml is None or not os.path.isfile(yaml_path):
        return float("nan"), float("nan")
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    if not cfg:
        return float("nan"), float("nan")
    return _cfg_float(cfg, "max_trans_e_m"), _cfg_float(cfg, "max_rot_e_deg")


def _optimized_pose_error_path(base, map_dir, traj_key):
    opt_dir = os.path.join(
        base, map_dir, traj_key, "{}_none".format(traj_key), "optimized_path_yaw"
    )
    path_yaw = os.path.join(opt_dir, POSE_E_PATH_YAW_NM)
    plain = os.path.join(opt_dir, POSE_E_NM)
    if os.path.isfile(path_yaw):
        return path_yaw
    if os.path.isfile(plain):
        return plain
    return None


def optimized_ours_cell(base, map_dir, traj_key):
    """
    Failure rate = fraction of poses with NaN rotation error (after thresholds),
    same definition as registration_stats for other methods.
    """
    err_fn = _optimized_pose_error_path(base, map_dir, traj_key)
    if err_fn is None:
        return fmt_cell(None, None)
    yaml_path = os.path.join(base, map_dir, traj_key, "analysis_cfg.yaml")
    max_t, max_r = _analysis_thresholds(yaml_path)
    try:
        _, rot_e = _load_pose_error(err_fn, max_t, max_r)
    except (OSError, ValueError, IndexError):
        return fmt_cell(None, None)
    total = len(rot_e)
    if total == 0:
        return fmt_cell(None, None)
    n_failed = sum(1 for v in rot_e if math.isnan(v))
    return fmt_cell((1.0 * n_failed) / total, total)


DEFAULT_MAPS = [
    ("trace_r2_a20", "r2-a20"),
    ("trace_r1_a30", "r1-a30"),
]

TRAJ_ORDER = [
    ("top", "Top"),
    ("diagonal", "Diagonal"),
    ("bottom", "Bottom"),
    ("left", "Left"),
    ("bigU", "BigU"),
    ("smallU", "SmallU"),
    ("mid", "Mid"),
]

COL_TYPES = [
    "none",
    "pc_det",
    "pc_trace",
    "gp_det",
    "gp_trace",
    "quad_det",
    "quad_trace",
    "optimized_path_yaw",
]


def fmt_cell(nonreg, n_total):
    if nonreg is None:
        return "X"
    try:
        nf = float(nonreg)
    except (TypeError, ValueError):
        return "X"
    if math.isnan(nf):
        return "X"
    try:
        nt = int(n_total)
    except (TypeError, ValueError):
        return "X"
    if nt == 0:
        return "X"
    pct = int(round(100 * nf))
    return r"{}\%".format(pct)


def row_for_traj(base, map_dir, traj_key, optimized_from_pose=True):
    csv_path = os.path.join(base, map_dir, traj_key, "registration_stats.csv")
    yaml_path = os.path.join(base, map_dir, traj_key, "analysis_cfg.yaml")
    if not os.path.isfile(csv_path):
        base_row = [fmt_cell(None, None)] * len(COL_TYPES)
        if optimized_from_pose:
            base_row[-1] = optimized_ours_cell(base, map_dir, traj_key)
        return base_row
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    by_name = {r["name"]: r for r in rows}
    types_map = {}
    if yaml is not None and os.path.isfile(yaml_path):
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
        types_map = cfg.get("types") or {}
    out = []
    col_types = COL_TYPES[:-1] if optimized_from_pose else COL_TYPES
    for t in col_types:
        keys = [k for k, v in types_map.items() if v == t]
        if not keys:
            keys = ["{}_{}".format(traj_key, t)]
        val = None
        nt = None
        for k in keys:
            if k in by_name:
                r = by_name[k]
                val = r.get("non_registered_ratio")
                nt = r.get("n_total")
                break
        if val is None:
            out.append(fmt_cell(None, None))
            continue
        if isinstance(val, str) and val.lower() == "nan":
            out.append("X")
            continue
        out.append(fmt_cell(val, nt))
    if optimized_from_pose:
        out.append(optimized_ours_cell(base, map_dir, traj_key))
    else:
        t = COL_TYPES[-1]
        keys = [k for k, v in types_map.items() if v == t]
        if not keys:
            keys = ["{}_{}".format(traj_key, t)]
        val = None
        nt = None
        for k in keys:
            if k in by_name:
                r = by_name[k]
                val = r.get("non_registered_ratio")
                nt = r.get("n_total")
                break
        if val is None:
            out.append(fmt_cell(None, None))
        elif isinstance(val, str) and val.lower() == "nan":
            out.append("X")
        else:
            out.append(fmt_cell(val, nt))
    return out


def emit_rows(base, maps, outfile, optimized_from_pose=True):
    lines = []
    for map_dir, map_label in maps:
        lines.append(r"\midrule")
        lines.append(r"\multirow{7}{*}{" + map_label + "}")
        for i, (traj_key, traj_disp) in enumerate(TRAJ_ORDER):
            cells = row_for_traj(base, map_dir, traj_key, optimized_from_pose=optimized_from_pose)
            row_inner = " & ".join([traj_disp.ljust(8)] + cells)
            if i == 0:
                lines.append(" & " + row_inner + r" \\")
            else:
                lines.append(" & " + row_inner + r" \\")
    text = "\n".join(lines) + "\n"
    if outfile:
        with open(outfile, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)


def emit_full_table(base, maps, outfile, optimized_from_pose=True):
    header = r"""\begin{table*}[t]
\centering
\caption{Registration failure rates (\% failed localizations). X denotes that no valid solution was returned.}
\label{tab:registration_failure}
\begin{tabular}{llrrrrrrrr}
\toprule
 & cfg. & No Info & PC-det & PC-Trace & GP-det & GP-Trace & Quad-det & Quad-Trace & optimized(ours) \\
"""
    body_lines = []
    for map_dir, map_label in maps:
        body_lines.append(r"\midrule")
        body_lines.append(r"\multirow{7}{*}{" + map_label + "}")
        for i, (traj_key, traj_disp) in enumerate(TRAJ_ORDER):
            cells = row_for_traj(base, map_dir, traj_key, optimized_from_pose=optimized_from_pose)
            row_inner = " & ".join([traj_disp.ljust(8)] + cells)
            if i == 0:
                body_lines.append(" & " + row_inner + r" \\")
            else:
                body_lines.append(" & " + row_inner + r" \\")
    footer = r"""\bottomrule
\end{tabular}
\end{table*}
"""
    text = header + "\n".join(body_lines) + "\n" + footer
    if outfile:
        with open(outfile, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)


def parse_maps(specs):
    """Each spec is 'trace_r2_a20:r2-a20' or just 'trace_r2_a20' (display = folder with underscores->hyphens)."""
    out = []
    for s in specs:
        s = s.strip()
        if not s:
            continue
        if ":" in s:
            d, lbl = s.split(":", 1)
            out.append((d.strip(), lbl.strip()))
        else:
            lbl = s.replace("trace_", "").replace("_", "-")
            out.append((s, lbl))
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_base = os.path.normpath(os.path.join(here, ".."))

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--base",
        default=default_base,
        help="act_map_exp root (parent of trace_* folders). Default: ../ from this script.",
    )
    p.add_argument(
        "--maps",
        nargs="*",
        default=None,
        metavar="DIR:LABEL",
        help="Map folders and LaTeX labels, e.g. trace_r2_a20:r2-a20. Default: built-in r2/r1 maps.",
    )
    p.add_argument(
        "--full-table",
        action="store_true",
        help="Emit complete table* environment (not just midrule rows).",
    )
    p.add_argument(
        "-o", "--output",
        default=None,
        help="Write to this file instead of stdout.",
    )
    p.add_argument(
        "--optimized-from-csv",
        action="store_true",
        help="Take optimized(ours) from registration_stats.csv instead of pose_errors under *_none/optimized_path_yaw/.",
    )
    args = p.parse_args()

    maps = parse_maps(args.maps) if args.maps else list(DEFAULT_MAPS)

    if yaml is None:
        print("Warning: PyYAML not installed; analysis_cfg.yaml will be ignored.", file=sys.stderr)

    opt_pose = not args.optimized_from_csv

    if args.full_table:
        emit_full_table(args.base, maps, args.output, optimized_from_pose=opt_pose)
    else:
        emit_rows(args.base, maps, args.output, optimized_from_pose=opt_pose)

    return 0


if __name__ == "__main__":
    sys.exit(main())
