#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner="${root_dir}/act_map_exp/scripts/run_planner_exp.py"
config="${FOV_REG_CFG:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/warehouse_all.yaml}"
defaults="${FOV_REG_DEFAULTS:-${root_dir}/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults.yaml}"

if [[ ! -f "${planner}" ]]; then
  echo "Missing registration script: ${planner}" >&2
  exit 1
fi
if [[ ! -f "${config}" ]]; then
  echo "Missing registration config: ${config}" >&2
  exit 1
fi
if [[ ! -f "${defaults}" ]]; then
  echo "Missing registration defaults: ${defaults}" >&2
  exit 1
fi

exec python3 "${planner}" "${config}" --defaults_yaml "${defaults}" "$@"
