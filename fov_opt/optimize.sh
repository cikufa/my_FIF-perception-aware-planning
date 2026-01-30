#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
script="${root_dir}/act_map_exp/scripts/run_fov_opt_rrt_none.sh"

if [[ ! -x "${script}" ]]; then
  echo "Missing optimizer script: ${script}" >&2
  exit 1
fi

exec "${script}" "$@"
