#!/usr/bin/env bash
# Analyze only warehouse_left, warehouse_mid, warehouse_right, warehouse_side;
# write summaries under a separate folder (default: .../analysis_trajs_left_mid_right_side).
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analyzer="${root_dir}/fov_opt/analyze_all.py"
trace_xyz="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt_xyz"
trace_full="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt"
wh_full="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/warehouse_all.yaml"
base_to="${root_dir}/act_map_exp/params/quad_traj_opt/base_analysis_cfg.yaml"
base_rrt="${root_dir}/act_map_exp/params/quad_rrt/base_analysis_cfg.yaml"

top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_full}}"
base_ana="${FOV_TRAJOPT_BASE_ANA_CFG:-}"
if [[ -z "${base_ana}" ]]; then
  if [[ -f "${base_to}" ]]; then
    base_ana="${base_to}"
  else
    base_ana="${base_rrt}"
  fi
fi

default_out="$(dirname "${trace_full}")/analysis_trajs_left_mid_right_side"
out_dir="${FOV_TRAJOPT_ANALYSIS_ARTIFACTS_DIR:-${default_out}}"
merge_from="${FOV_TRAJOPT_MERGE_OPTIMIZED_FROM:-${trace_xyz}}"

exec python3 "${analyzer}" \
  --top-dir "${top_dir}" \
  --warehouse-all-yaml "${wh_full}" \
  --base-ana-cfg "${base_ana}" \
  --merge-optimized-from "${merge_from}" \
  --analysis-artifacts-dir "${out_dir}" \
  --include-trajs "warehouse_left,warehouse_mid,warehouse_right,warehouse_side" \
  "$@"
