#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner="${root_dir}/act_map_exp/scripts/run_planner_exp.py"
config="${FOV_REG_CFG:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/warehouse_all.yaml}"
defaults="${FOV_REG_DEFAULTS:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults.yaml}"

mode="variants"
along=""
extra_args=()

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--variants|--optimized] [--along-path|--normal] [extra run_planner_exp args]
  --variants     Register all non-optimized variations (default).
  --optimized    Register optimized results only.
  --along-path   Use along-path optimized output (optimized mode only).
  --normal       Use normal optimized output (optimized mode only).
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --variants)
      mode="variants"
      ;;
    --optimized|--optimized-only)
      mode="optimized"
      ;;
    --along-path|--along_path)
      along="along"
      ;;
    --normal)
      along="normal"
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

if [[ "$mode" == "variants" && "$along" == "along" ]]; then
  echo "--along-path is only valid with --optimized" >&2
  exit 1
fi

cmd=(python3 "$planner" "$config" --defaults_yaml "$defaults")
if [[ "$mode" == "variants" ]]; then
  cmd+=(--skip_optimized)
else
  cmd+=(--only_optimized)
  if [[ "$along" == "along" ]]; then
    cmd+=(--along_path)
  fi
fi
cmd+=("${extra_args[@]}")

exec "${cmd[@]}"
