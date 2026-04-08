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
Usage: $(basename "$0") [--xyz] [--full] [analyze_all.py args...]

  Default: --top-dir traj_opt + warehouse_all.yaml (variant registration results).
  --xyz:    traj_opt_xyz + warehouse_all_xyz_only.yaml (FoV-optimized outputs under xyz none folders).
  --full:   same as default here (traj_opt + warehouse_all); use if you only run the --full register pipeline.

Run twice (default then --xyz) to refresh plots for both trees.

Further args are passed to analyze_all.py (e.g. --plt-min-ratio 0.2).

Env: FOV_TRAJOPT_TOP_DIR, FOV_TRAJOPT_WAREHOUSE_YAML, FOV_TRAJOPT_BASE_ANA_CFG override defaults.
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

while [[ $# -gt 0 ]]; do
  case "${1:-}" in
    --xyz)
      top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_xyz}}"
      wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_xyz}}"
      shift
      ;;
    --full)
      top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_full}}"
      wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_full}}"
      shift
      ;;
    *)
      break
      ;;
  esac
done

exec python3 "${analyzer}" \
  --top-dir "${top_dir}" \
  --warehouse-all-yaml "${wh_yaml}" \
  --base-ana-cfg "${base_ana}" \
  "$@"
