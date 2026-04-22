#!/usr/bin/env bash
# RRT FoV runs only. Traj-opt uses fov_opt/optimize_trajopt.sh.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
default_manifold_bin="/home/shekoufeh/fov/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory"
default_best_params="${root_dir}/fov_opt/optuna/optuna_best_params.yaml"
default_esdf="/home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_voxblox/tsdf_esdf_max10.vxblx"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--map r2_a20|r1_a30] [optimize.sh args...]

  Default: FoV optimize under trace/trace_rrt_<map>
           (RRT none baselines, output in *_none/optimized).
  --map:   choose dataset/map root. Default: r2_a20.
  --output-dir-name NAME:
           forwarded to optimize.sh; changes the output subdirectory name.
  --with-ray-occlusion / --with-occlusion:
           enable ESDF ray-based occlusion filtering (default).
  --without-ray-occlusion / --without-occlusion:
           disable ESDF ray-based occlusion filtering.
  --compare-ray-occlusion:
           forwarded to optimize.sh; writes sibling with/without-ray runs.
  --esdf:  override ESDF map path (also enables ray-based occlusion).

Auto-configures:
  - trace root
  - points3D.txt
  - best params YAML
  - ESDF map
  - manifold binary

Forwards all other arguments to optimize.sh (e.g. --summarize or explicit view names).

Env: same as optimize.sh for tuning/summary options. This wrapper hardcodes
     FOV_MANIFOLD_BIN, defaults FOV_BEST_PARAMS_YAML to the local optuna best
     params file, and enables FOV_OPT_ESDF_PATH by default unless disabled.
USAGE
}

map_name="${FOV_RRT_MAP:-r2_a20}"
occlusion_mode="with"
esdf_override=""
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
    --with-ray-occlusion|--with-occlusion)
      occlusion_mode="with"
      ;;
    --without-ray-occlusion|--without-occlusion)
      occlusion_mode="without"
      ;;
    --esdf|--esdf-map|--esdf_map)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --esdf" >&2
        exit 2
      fi
      esdf_override="$2"
      occlusion_mode="with"
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
if [[ -z "${FOV_BEST_PARAMS_YAML:-}" && -f "${default_best_params}" ]]; then
  export FOV_BEST_PARAMS_YAML="${default_best_params}"
fi

if [[ "${occlusion_mode}" == "with" ]]; then
  esdf_path="${esdf_override:-${FOV_OPT_ESDF_PATH:-${default_esdf}}}"
  if [[ ! -f "${esdf_path}" ]]; then
    echo "ESDF map not found: ${esdf_path}" >&2
    exit 1
  fi
  export FOV_OPT_ESDF_PATH="${esdf_path}"
else
  unset FOV_OPT_ESDF_PATH
fi

exec "${root_dir}/fov_opt/optimize.sh" \
  --trace-root "${trace_root}" \
  --points3d "${points3d}" \
  "${extra_args[@]}"
