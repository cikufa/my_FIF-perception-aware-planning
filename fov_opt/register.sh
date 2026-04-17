#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner="${root_dir}/act_map_exp/scripts/run_planner_exp.py"
config="${FOV_REG_CFG:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/warehouse_all.yaml}"
defaults="${FOV_REG_DEFAULTS:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults.yaml}"

mode=""
with_along_path=true
dataset=""
defaults_override=""
extra_args=()
dataset_mode="single"
defaults_r1="${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r1_a30.yaml"
defaults_r2="${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r2_a20.yaml"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--variants|--optimized|--all] [--no-along-path] [--dataset NAME] [extra run_planner_exp args]
  --variants     Register all non-optimized variations.
  --optimized    Register optimized results only.
  --all          Register both non-optimized variations and optimized results.
  --no-along-path  Disable along_path baseline generation for --variants.
  --dataset      r2_a20, r1_a30, or both (selects defaults file[s]).
  --defaults-yaml PATH  Override defaults yaml.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --variants)
      mode="variants"
      ;;
    --optimized)
      mode="optimized"
      ;;
    --all)
      mode="all"
      ;;
    --along-path|--along_path)
      with_along_path=true
      ;;
    --no-along-path|--no_along_path)
      with_along_path=false
      ;;
    --normal|--both)
      echo "Error: only optimized_path_yaw is supported now (no --normal/--both)." >&2
      exit 2
      ;;
    --dataset)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --dataset" >&2
        exit 2
      fi
      dataset="$2"
      shift
      ;;
    --defaults-yaml|--defaults_yaml)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --defaults-yaml" >&2
        exit 2
      fi
      defaults_override="$2"
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

if [[ -z "$mode" ]]; then
  mode="variants"
fi

# Default to all frames unless caller explicitly provides max_reg_images.
default_max_reg=()
has_max_reg=false
for arg in "${extra_args[@]}"; do
  case "$arg" in
    --max_reg_images|--max-reg-images|--max_reg_images=*|--max-reg-images=*)
      has_max_reg=true
      break
      ;;
  esac
done
if [[ "${has_max_reg}" == "false" ]]; then
  default_max_reg=(--max_reg_images 0)
fi

if [[ -n "${dataset}" ]]; then
  case "${dataset}" in
    r2_a20)
      defaults="${defaults_r2}"
      ;;
    r1_a30)
      defaults="${defaults_r1}"
      ;;
    both)
      dataset_mode="both"
      ;;
    *)
      echo "Unknown dataset: ${dataset} (expected r2_a20, r1_a30, or both)" >&2
      exit 2
      ;;
  esac
fi

if [[ -n "${defaults_override}" ]]; then
  if [[ "${dataset_mode}" == "both" ]]; then
    echo "Warning: --dataset both with --defaults-yaml uses the same defaults for both runs." >&2
    defaults_r1="${defaults_override}"
    defaults_r2="${defaults_override}"
  else
    defaults="${defaults_override}"
  fi
fi

run_variants() {
  local defaults_file="$1"
  local cmd=(python3 "$planner" "$config" --defaults_yaml "$defaults_file" --skip_optimized)
  if [[ "${with_along_path}" == "true" ]]; then
    cmd+=(--along_path)
  fi
  cmd+=("${default_max_reg[@]}")
  cmd+=("${extra_args[@]}")
  "${cmd[@]}"
}

run_optimized() {
  local defaults_file="$1"
  local cmd=(python3 "$planner" "$config" --defaults_yaml "$defaults_file" --only_optimized)
  cmd+=(--along_path)
  cmd+=("${default_max_reg[@]}")
  cmd+=("${extra_args[@]}")
  "${cmd[@]}"
}

case "$mode" in
  variants)
    if [[ "${dataset_mode}" == "both" ]]; then
      run_variants "${defaults_r1}"
      run_variants "${defaults_r2}"
      exit 0
    fi
    run_variants "${defaults}"
    exit 0
    ;;
  optimized)
    if [[ "${dataset_mode}" == "both" ]]; then
      run_optimized "${defaults_r1}"
      run_optimized "${defaults_r2}"
      exit 0
    fi
    run_optimized "${defaults}"
    exit 0
    ;;
  all)
    if [[ "${dataset_mode}" == "both" ]]; then
      run_variants "${defaults_r1}"
      run_variants "${defaults_r2}"
    else
      run_variants "${defaults}"
    fi
    if [[ "${dataset_mode}" == "both" ]]; then
      run_optimized "${defaults_r1}"
      run_optimized "${defaults_r2}"
    else
      run_optimized "${defaults}"
    fi
    ;;
esac
