#!/usr/bin/env bash
# Trajectory-optimization FoV runs only. RRT uses fov_opt/optimize.sh unchanged.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
trace_xyz="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt_xyz"
trace_full="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt"
points3d="${root_dir}/act_map_exp/exp_data/warehouse_base_model_r1_a30/sparse/0/points3D.txt"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--full] [optimize.sh args...]

  Default: FoV optimize under traj_opt_xyz (xyz-only / none baselines, same role as RRT top_none).
  --full:  use traj_opt (full pose+yaw planned traces).

Forwards all other arguments to optimize.sh (e.g. --along-path --summarize, or explicit view names).

Env: same as optimize.sh (FOV_TUNE_CONFIG, FOV_BEST_PARAMS_YAML, FOV_MANIFOLD_BIN, ...).
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

trace_root="${trace_xyz}"
if [[ "${1:-}" == "--full" ]]; then
  trace_root="${trace_full}"
  shift
fi

exec "${root_dir}/fov_opt/optimize.sh" --trace-root "${trace_root}" --points3d "${points3d}" "$@"
