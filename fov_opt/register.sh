#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner="${root_dir}/act_map_exp/scripts/run_planner_exp.py"
config="${FOV_REG_CFG:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/warehouse_all.yaml}"
defaults="${FOV_REG_DEFAULTS:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults.yaml}"

mode=""
along=""
dataset=""
defaults_override=""
extra_args=()

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--variants|--optimized|--all] [--normal|--along-path|--both] [extra run_planner_exp args]
  --variants     Register all non-optimized variations.
  --optimized    Register optimized results only.
  --all          Register both non-optimized variations and optimized results.
  --normal       Use normal optimized output (optimized/all modes only).
  --along-path   Use along-path optimized output (optimized/all modes only).
  --both         Register both normal and along-path optimized outputs.
  --dataset      r2_a20 or r1_a30 (selects defaults file).
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
      along="along"
      ;;
    --normal)
      along="normal"
      ;;
    --both)
      along="both"
      ;;
    --dataset)
      dataset="$2"
      shift
      ;;
    --defaults-yaml|--defaults_yaml)
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

if [[ -n "${dataset}" ]]; then
  case "${dataset}" in
    r2_a20)
      defaults="${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r2_a20.yaml"
      ;;
    r1_a30)
      defaults="${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r1_a30.yaml"
      ;;
    *)
      echo "Unknown dataset: ${dataset} (expected r2_a20 or r1_a30)" >&2
      exit 2
      ;;
  esac
fi

if [[ -n "${defaults_override}" ]]; then
  defaults="${defaults_override}"
fi

if [[ "$mode" == "variants" && ( "$along" == "along" || "$along" == "both" ) ]]; then
  echo "--along-path/--both is only valid with --optimized or --all" >&2
  exit 1
fi

base_cmd=(python3 "$planner" "$config" --defaults_yaml "$defaults")

run_variants() {
  local cmd=("${base_cmd[@]}" --skip_optimized)
  cmd+=("${extra_args[@]}")
  "${cmd[@]}"
}

run_optimized() {
  local mode="$1"
  local cmd=("${base_cmd[@]}" --only_optimized)
  if [[ "$mode" == "along" ]]; then
    cmd+=(--along_path)
  fi
  cmd+=("${extra_args[@]}")
  "${cmd[@]}"
}

case "$mode" in
  variants)
    cmd=("${base_cmd[@]}" --skip_optimized "${extra_args[@]}")
    exec "${cmd[@]}"
    ;;
  optimized)
    if [[ "$along" == "both" ]]; then
      run_optimized "normal"
      run_optimized "along"
      exit 0
    fi
    run_optimized "$along"
    exit 0
    ;;
  all)
    run_variants
    if [[ "$along" == "both" ]]; then
      run_optimized "normal"
      run_optimized "along"
    else
      run_optimized "$along"
    fi
    ;;
esac
