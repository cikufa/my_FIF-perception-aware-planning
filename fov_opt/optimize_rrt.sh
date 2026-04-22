#!/usr/bin/env bash
# RRT FoV runs only. Traj-opt uses fov_opt/optimize_trajopt.sh.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
default_manifold_bin="/home/shekoufeh/fov/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory"
default_esdf="/home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_voxblox/tsdf_esdf_max10.vxblx"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--map r2_a20|r1_a30] [optimize.sh args...]

  Default: FoV optimize under trace/trace_rrt_<map>
           (RRT none baselines, output in *_none/optimized_path_yaw).
  --map:   choose dataset/map root. Default: r2_a20.

Auto-configures:
  - trace root
  - points3D.txt
  - ESDF map
  - manifold binary

Forwards all other arguments to optimize.sh (e.g. --summarize or explicit view names).

Env: same as optimize.sh for tuning/summary options. This wrapper hardcodes
     FOV_MANIFOLD_BIN and FOV_OPT_ESDF_PATH.
USAGE
}

map_name="${FOV_RRT_MAP:-r2_a20}"
extra_args=()

while [[ $# -gt 0 ]]; do
  case "${1:-}" in
    --map|--dataset)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --map" >&2
        exit 2
      fi
      map_name="$2"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      extra_args+=("$1")
      ;;
  esac
  shift
done

case "${map_name}" in
  r2_a20|r1_a30)
    ;;
  *)
    echo "Unknown map: ${map_name} (expected r2_a20 or r1_a30)" >&2
    exit 2
    ;;
esac

trace_root="${root_dir}/act_map_exp/trace/trace_rrt_${map_name}"
points3d="${root_dir}/act_map_exp/exp_data/warehouse_base_model_${map_name}/sparse/0/points3D.txt"

if [[ ! -d "${trace_root}" ]]; then
  echo "Trace root not found: ${trace_root}" >&2
  exit 1
fi
if [[ ! -f "${points3d}" ]]; then
  echo "points3D.txt not found: ${points3d}" >&2
  exit 1
fi

if [[ ! -x "${default_manifold_bin}" ]]; then
  echo "Hardcoded manifold binary not found: ${default_manifold_bin}" >&2
  exit 1
fi
export FOV_MANIFOLD_BIN="${default_manifold_bin}"

if [[ ! -f "${default_esdf}" ]]; then
  echo "Hardcoded ESDF map not found: ${default_esdf}" >&2
  exit 1
fi
export FOV_OPT_ESDF_PATH="${default_esdf}"

exec "${root_dir}/fov_opt/optimize.sh" \
  --trace-root "${trace_root}" \
  --points3d "${points3d}" \
  "${extra_args[@]}"
