#!/usr/bin/env bash
# Trajectory-optimization registration only. RRT uses fov_opt/register.sh unchanged.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner="${root_dir}/act_map_exp/scripts/run_planner_exp.py"
# Planner bases for traj_opt experiments (full planned trajs under traj_opt/).
cfg_std="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/warehouse_all.yaml"

def_variants="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/run_planner_defaults_reg_variants.yaml"
def_optimized_xyz="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/run_planner_defaults_reg_optimized_xyz.yaml"
def_full="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/run_planner_defaults_fov_full.yaml"

config="${FOV_TRAJOPT_REG_CFG:-${cfg_std}}"
defaults_variants="${FOV_TRAJOPT_REG_DEFAULTS_VARIANTS:-${def_variants}}"
defaults_optimized="${FOV_TRAJOPT_REG_DEFAULTS_OPTIMIZED:-${def_optimized_xyz}}"
# Used when --full or as FOV_TRAJOPT_REG_DEFAULTS target for full traj_opt tree.
defaults_full="${FOV_TRAJOPT_REG_DEFAULTS:-${def_full}}"

mode=""
with_along_path=true
defaults_override=""
extra_args=()
use_full=false

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--full] [--variants|--optimized|--all] [--no-along-path] [extra run_planner_exp args]

  Default (no --full):
    --variants   top_outdir = trace_trajopt_r1_a30/traj_opt (all planned trajs / info variants).
                 traj_opt_xyz is NOT used here (that tree is only for FoV optimization inputs).
    --optimized  Registers .../traj_opt_xyz/.../optimized_path_yaw (output of optimize_trajopt.sh).
    --all        Runs variants then optimized (each with the correct defaults yaml).

  --full: variants and optimized both use traj_opt + run_planner_defaults_fov_full.yaml
          (full-pose traj-opt tree; FoV optimized folders live under traj_opt).

Override planner yaml: FOV_TRAJOPT_REG_CFG (default: warehouse_all.yaml).
Override defaults: FOV_TRAJOPT_REG_DEFAULTS_VARIANTS, FOV_TRAJOPT_REG_DEFAULTS_OPTIMIZED,
  or FOV_TRAJOPT_REG_DEFAULTS with --full. Or --defaults-yaml (applies to every run in this invocation).
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --full)
      use_full=true
      ;;
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

if [[ -n "${defaults_override}" ]]; then
  defaults_variants="${defaults_override}"
  defaults_optimized="${defaults_override}"
elif [[ "${use_full}" == "true" ]]; then
  defaults_variants="${defaults_full}"
  defaults_optimized="${defaults_full}"
fi

if [[ -z "$mode" ]]; then
  mode="variants"
fi

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
    run_variants "${defaults_variants}"
    ;;
  optimized)
    run_optimized "${defaults_optimized}"
    ;;
  all)
    run_variants "${defaults_variants}"
    run_optimized "${defaults_optimized}"
    ;;
esac
