#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
script="${root_dir}/fov_opt/optuna_fov_tune.py"
config="${FOV_TUNE_CONFIG:-${root_dir}/fov_opt/optuna_fov_tune.yaml}"

mode=""
along=""
extra_args=()

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--tune|--fixed] [--along-path|--normal] [extra optuna args]
  --tune       Run tuning mode.
  --fixed      Run fixed mode (auto-loads best_params_yaml if fixed_params is empty).
  --along-path Force along_path=true.
  --normal     Force along_path=false.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tune)
      mode="tune"
      ;;
    --fixed)
      mode="fixed"
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

cmd=(python3 "$script" --config "$config")
if [[ -n "$mode" ]]; then
  cmd+=(--mode "$mode")
fi
if [[ "$along" == "along" ]]; then
  cmd+=(--along-path)
elif [[ "$along" == "normal" ]]; then
  cmd+=(--normal)
fi
cmd+=("${extra_args[@]}")

exec "${cmd[@]}"
