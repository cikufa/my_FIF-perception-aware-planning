#!/usr/bin/env bash
# Trajectory-optimization analysis only. RRT uses fov_opt/analyze.sh unchanged.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analyzer="${root_dir}/fov_opt/analyze_all.py"
trace_xyz="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt_xyz"
trace_full="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt"
wh_xyz="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/warehouse_all_xyz_only.yaml"
wh_full="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/warehouse_all.yaml"
base_rrt="${root_dir}/act_map_exp/params/quad_rrt/base_analysis_cfg.yaml"
base_to="${root_dir}/act_map_exp/params/quad_traj_opt/base_analysis_cfg.yaml"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--xyz] [--full] [--no-merge] [analyze_all.py args...]

  Default: traj_opt + warehouse_all.yaml, and pulls optimized_path_yaw TE/RE from
            traj_opt_xyz (same warehouse_* names). FoV outputs live only under xyz.

  --xyz:       analyze traj_opt_xyz only (no merge; none + optimized there).
  --full:      traj_opt only, single-tree pipeline; disables merge (same as --no-merge).
  --no-merge:  do not pass --merge-optimized-from (e.g. custom top_dir layout).

Env: FOV_TRAJOPT_TOP_DIR, FOV_TRAJOPT_WAREHOUSE_YAML, FOV_TRAJOPT_BASE_ANA_CFG,
     FOV_TRAJOPT_MERGE_OPTIMIZED_FROM (default: traj_opt_xyz), FOV_TRAJOPT_NO_MERGE_OPTIMIZED=1
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_full}}"
wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_full}}"
base_ana="${FOV_TRAJOPT_BASE_ANA_CFG:-}"
if [[ -z "${base_ana}" ]]; then
  if [[ -f "${base_to}" ]]; then
    base_ana="${base_to}"
  else
    base_ana="${base_rrt}"
  fi
fi

use_xyz_only=false
no_merge=false

while [[ $# -gt 0 ]]; do
  case "${1:-}" in
    --xyz)
      top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_xyz}}"
      wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_xyz}}"
      use_xyz_only=true
      shift
      ;;
    --full)
      top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_full}}"
      wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_full}}"
      no_merge=true
      shift
      ;;
    --no-merge)
      no_merge=true
      shift
      ;;
    *)
      break
      ;;
  esac
done

merge_arg=()
if [[ "${FOV_TRAJOPT_NO_MERGE_OPTIMIZED:-}" == "1" ]]; then
  no_merge=true
fi
if [[ "${use_xyz_only}" == "false" && "${no_merge}" == "false" ]]; then
  merge_from="${FOV_TRAJOPT_MERGE_OPTIMIZED_FROM:-${trace_xyz}}"
  merge_arg=(--merge-optimized-from "${merge_from}")
fi

exec python3 "${analyzer}" \
  --top-dir "${top_dir}" \
  --warehouse-all-yaml "${wh_yaml}" \
  --base-ana-cfg "${base_ana}" \
  "${merge_arg[@]}" \
  "$@"
