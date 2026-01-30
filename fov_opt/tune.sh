#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
script="${root_dir}/fov_opt/optuna_fov_tune.py"
config="${FOV_TUNE_CONFIG:-${root_dir}/fov_opt/optuna_fov_tune.yaml}"

if [[ ! -f "${script}" ]]; then
  echo "Missing tuning script: ${script}" >&2
  exit 1
fi
if [[ ! -f "${config}" ]]; then
  echo "Missing tuning config: ${config}" >&2
  exit 1
fi

exec python3 "${script}" --config "${config}" "$@"
