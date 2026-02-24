#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
script="${root_dir}/fov_opt/optuna_fov_tune.py"
config="${FOV_TUNE_CONFIG:-${root_dir}/fov_opt/optuna_fov_tune.yaml}"

mode=""
along=""
dataset=""
workers=1
storage_override=""
study_override=""
extra_args=()

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--tune] [--along-path|--normal] [extra optuna args]
  --tune       Run tuning mode.
  --along-path Force along_path=true.
  --normal     Force along_path=false.
  --dataset    r2_a20 or r1_a30 (selects config + default storage).
  --workers N  Run N Optuna workers (multi-process, shared storage).
  --storage S  Override Optuna storage URL (e.g. sqlite:////path/optuna.db).
  --study-name NAME Override study name (shared across workers).
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tune)
      mode="tune"
      ;;
    --fixed)
      echo "Error: --fixed was removed. Use ./optimize.sh + ./register.sh instead." >&2
      exit 2
      ;;
    --along-path|--along_path)
      along="along"
      ;;
    --normal)
      along="normal"
      ;;
    --dataset)
      dataset="$2"
      shift
      ;;
    --workers)
      workers="$2"
      shift
      ;;
    --storage)
      storage_override="$2"
      shift
      ;;
    --study-name)
      study_override="$2"
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

if [[ -n "${dataset}" ]]; then
  case "${dataset}" in
    r2_a20)
      config="${root_dir}/fov_opt/optuna_fov_tune.yaml"
      ;;
    r1_a30)
      config="${root_dir}/fov_opt/optuna_fov_tune_r1_a30.yaml"
      ;;
    *)
      echo "Unknown dataset: ${dataset} (expected r2_a20 or r1_a30)" >&2
      exit 2
      ;;
  esac
fi

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

if [[ "$workers" -le 1 ]]; then
  if [[ -n "$storage_override" ]]; then
    cmd+=(--storage "$storage_override")
  fi
  if [[ -n "$study_override" ]]; then
    cmd+=(--study-name "$study_override")
  fi
  exec "${cmd[@]}"
fi

default_storage="sqlite:////home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/trace_r2_a20/optuna.db"
if [[ "${dataset}" == "r1_a30" ]]; then
  default_storage="sqlite:////home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/trace_r1_a30/optuna.db"
fi
storage="${storage_override:-$default_storage}"

workers=$((workers))
if [[ "$workers" -lt 1 ]]; then
  workers=1
fi

pids=()
for ((i=0; i<workers; i++)); do
  worker_cmd=("${cmd[@]}")
  worker_cmd+=(--storage "$storage")
  if [[ -n "$study_override" ]]; then
    worker_cmd+=(--study-name "$study_override")
  fi
  "${worker_cmd[@]}" &
  pids+=("$!")
done

for pid in "${pids[@]}"; do
  wait "$pid"
done
