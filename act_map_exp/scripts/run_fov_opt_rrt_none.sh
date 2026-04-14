#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd "${script_dir}/../../.." && pwd)"

manifold_bin="${FOV_MANIFOLD_BIN:-${root_dir}/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory}"
default_trace_root="${root_dir}/my_FIF-perception-aware-planning/act_map_exp/trace_r2_a20"
trace_root="${FOV_TRACE_ROOT:-${default_trace_root}}"
default_points3d_r2="${root_dir}/my_FIF-perception-aware-planning/act_map_exp/exp_data/warehouse_base_model_r2_a20/sparse/0/points3D.txt"
default_points3d_r1="${root_dir}/my_FIF-perception-aware-planning/act_map_exp/exp_data/warehouse_base_model_r1_a30/sparse/0/points3D.txt"
default_points3d_legacy="${root_dir}/my_FIF-perception-aware-planning/act_map_exp/localization/warehouse_base/sparse/0/points3D.txt"
points3d="${FOV_POINTS3D:-${default_points3d_r2}}"

if [[ ! -x "${manifold_bin}" ]]; then
  echo "Missing manifold binary: ${manifold_bin}" >&2
  exit 1
fi

if [[ ! -f "${points3d}" ]]; then
  echo "Missing points3D.txt: ${points3d}" >&2
  exit 1
fi

run_with_timing() {
  local out_dir="$1"
  shift
  local runtime_file="${out_dir}/optimization_time_sec.txt"
  python3 - "${out_dir}" "${runtime_file}" "$@" <<'PY'
import os
import subprocess
import sys
import time

out_dir = sys.argv[1]
runtime_file = sys.argv[2]
cmd = sys.argv[3:]

start = time.time()
proc = subprocess.Popen(cmd)
ret = proc.wait()
elapsed = time.time() - start

os.makedirs(out_dir, exist_ok=True)
with open(runtime_file, "w") as f:
    f.write("{:.6f}\n".format(elapsed))

sys.exit(ret)
PY
}

run_along_path_opt() {
  local input_dir="$1"
  local output_dir="$2"
  local points3d="$3"
  local along_path_input="${input_dir}/along_path/stamped_Twc_path_yaw.txt"
  if [[ -f "${along_path_input}" && -z "${FOV_OPT_WARM_START:-}" && -z "${FOV_OPT_WARM_START_FILE:-}" ]]; then
    FOV_OPT_WARM_START=1 FOV_OPT_WARM_START_FILE="${along_path_input}" \
      run_with_timing "${output_dir}" "${manifold_bin}" "${input_dir}" "${output_dir}" 1 "${points3d}"
  else
    run_with_timing "${output_dir}" "${manifold_bin}" "${input_dir}" "${output_dir}" 1 "${points3d}"
  fi
}

cleanup_quiver_metrics() {
  local output_dir="$1"
  if [[ "${FOV_OPT_KEEP_METRICS:-0}" == "1" ]]; then
    return
  fi
  rm -f "${output_dir}"/quivers*_metrics.txt 2>/dev/null || true
}

mode="along"
views=()
dataset=""
trace_root_override=""
points3d_override=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --along-path)
      mode="along"
      shift
      ;;
    --normal|--both)
      echo "Error: only optimized_path_yaw is supported now (use --along-path or no mode)." >&2
      exit 2
      ;;
    --dataset)
      dataset="$2"
      shift 2
      ;;
    --trace-root|--trace_root)
      trace_root_override="$2"
      shift 2
      ;;
    --points3d|--points3d-file|--points3d_file)
      points3d_override="$2"
      shift 2
      ;;
    --help|-h)
      echo "Usage: $0 [--along-path] [view1 view2 ...]" >&2
      echo "Default views: all subdirs in the trace root that contain <view>_none." >&2
      echo "Options:" >&2
      echo "  --dataset r2_a20|r1_a30" >&2
      echo "  --trace-root PATH" >&2
      echo "  --points3d PATH" >&2
      exit 0
      ;;
    *)
      views+=("$1")
      shift
      ;;
  esac
done
if [[ -n "${dataset}" ]]; then
  case "${dataset}" in
    r2_a20)
      trace_root="${root_dir}/my_FIF-perception-aware-planning/act_map_exp/trace_r2_a20"
      points3d="${default_points3d_r2}"
      ;;
    r1_a30)
      trace_root="${root_dir}/my_FIF-perception-aware-planning/act_map_exp/trace_r1_a30"
      points3d="${default_points3d_r1}"
      ;;
    *)
      echo "Unknown dataset: ${dataset} (expected r2_a20 or r1_a30)" >&2
      exit 2
      ;;
  esac
fi
if [[ -n "${trace_root_override}" ]]; then
  trace_root="${trace_root_override}"
fi
if [[ -n "${points3d_override}" ]]; then
  points3d="${points3d_override}"
fi
if [[ ! -f "${points3d}" && -f "${default_points3d_legacy}" ]]; then
  points3d="${default_points3d_legacy}"
fi
if [[ ${#views[@]} -eq 0 ]]; then
  if [[ ! -d "${trace_root}" ]]; then
    echo "Missing trace root: ${trace_root}" >&2
    exit 1
  fi
  mapfile -t views < <(find "${trace_root}" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
  if [[ ${#views[@]} -eq 0 ]]; then
    echo "No view directories found under ${trace_root}" >&2
    exit 1
  fi
  filtered=()
  for view in "${views[@]}"; do
    if [[ -d "${trace_root}/${view}/${view}_none" ]]; then
      filtered+=("${view}")
    fi
  done
  views=("${filtered[@]}")
  if [[ ${#views[@]} -eq 0 ]]; then
    echo "No <view>_none inputs found under ${trace_root}" >&2
    exit 1
  fi
fi

for view in "${views[@]}"; do
  input_dir="${trace_root}/${view}/${view}_none"
  output_dir_path_yaw="${input_dir}/optimized_path_yaw"
  if [[ ! -d "${input_dir}" ]]; then
    echo "Missing input dir: ${input_dir}" >&2
    continue
  fi
  case "${mode}" in
    along)
      mkdir -p "${output_dir_path_yaw}"
      run_along_path_opt "${input_dir}" "${output_dir_path_yaw}" "${points3d}"
      cleanup_quiver_metrics "${output_dir_path_yaw}"
      ;;
  esac
done
