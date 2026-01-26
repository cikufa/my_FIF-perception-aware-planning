#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd "${script_dir}/../../.." && pwd)"
gen_path_yaw="${script_dir}/generate_path_yaw.py"

manifold_bin="${FOV_MANIFOLD_BIN:-${root_dir}/My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory}"
trace_root="${FOV_TRACE_ROOT:-${root_dir}/my_FIF-perception-aware-planning/act_map_exp/trace}"
points3d="${FOV_POINTS3D:-${root_dir}/my_FIF-perception-aware-planning/act_map_exp/localization/warehouse_base/sparse/0/points3D.txt}"

if [[ ! -x "${manifold_bin}" ]]; then
  echo "Missing manifold binary: ${manifold_bin}" >&2
  exit 1
fi

if [[ ! -f "${points3d}" ]]; then
  echo "Missing points3D.txt: ${points3d}" >&2
  exit 1
fi

mode="normal"
views=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --along-path)
      mode="along"
      shift
      ;;
    --both)
      mode="both"
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--along-path|--both] [top diagonal bottom ...]" >&2
      exit 0
      ;;
    *)
      views+=("$1")
      shift
      ;;
  esac
done
if [[ ${#views[@]} -eq 0 ]]; then
  views=(top diagonal bottom)
fi

for view in "${views[@]}"; do
  input_dir="${trace_root}/${view}/${view}_none"
  output_dir="${input_dir}/optimized"
  output_dir_path_yaw="${input_dir}/optimized_path_yaw"
  if [[ ! -d "${input_dir}" ]]; then
    echo "Missing input dir: ${input_dir}" >&2
    continue
  fi
  case "${mode}" in
    normal)
      mkdir -p "${output_dir}"
      "${manifold_bin}" "${input_dir}" "${output_dir}" 0 "${points3d}"
      ;;
    along)
      if [[ -f "${gen_path_yaw}" ]]; then
        python3 "${gen_path_yaw}" --input_dir "${input_dir}"
      fi
      mkdir -p "${output_dir_path_yaw}"
      "${manifold_bin}" "${input_dir}" "${output_dir_path_yaw}" 1 "${points3d}"
      ;;
    both)
      if [[ -f "${gen_path_yaw}" ]]; then
        python3 "${gen_path_yaw}" --input_dir "${input_dir}"
      fi
      mkdir -p "${output_dir}" "${output_dir_path_yaw}"
      "${manifold_bin}" "${input_dir}" "${output_dir}" 0 "${points3d}"
      "${manifold_bin}" "${input_dir}" "${output_dir_path_yaw}" 1 "${points3d}"
      ;;
  esac
done
