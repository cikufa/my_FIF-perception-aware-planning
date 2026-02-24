#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
script="${root_dir}/act_map_exp/scripts/run_fov_opt_rrt_none.sh"
config="${FOV_TUNE_CONFIG:-${root_dir}/fov_opt/optuna_fov_tune.yaml}"
export FOV_TUNE_CONFIG="${config}"

if [[ ! -x "${script}" ]]; then
  echo "Missing optimizer script: ${script}" >&2
  exit 1
fi

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--normal|--along-path|--both] [--dataset NAME] [--trace-root PATH] [view1 view2 ...]
  --normal      Run normal optimization (default).
  --along-path  Run along-path optimization.
  --both        Run both normal and along-path optimization outputs.
  --dataset     r2_a20 or r1_a30 (sets trace root and points3d defaults).
  --trace-root  Override trace root directory.
  --points3d    Override points3D.txt path.
  --config      Override optuna_fov_tune.yaml.

Reads best params from optuna results (best_params_yaml in optuna_fov_tune.yaml).
If missing, falls back to initial_params/fixed_params in optuna_fov_tune.yaml.
USAGE
}

mode="normal"
views=()
dataset=""
trace_root_override=""
points3d_override=""
config_override=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --normal)
      mode="normal"
      ;;
    --along-path|--along_path)
      mode="along"
      ;;
    --both)
      mode="both"
      ;;
    --dataset)
      dataset="$2"
      shift
      ;;
    --trace-root|--trace_root)
      trace_root_override="$2"
      shift
      ;;
    --points3d|--points3d-file|--points3d_file)
      points3d_override="$2"
      shift
      ;;
    --config|--tune-config|--tune_config)
      config_override="$2"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      views+=("$1")
      ;;
  esac
  shift
done

if [[ -n "${config_override}" ]]; then
  config="${config_override}"
  export FOV_TUNE_CONFIG="${config}"
fi

if [[ -n "${dataset}" ]]; then
  case "${dataset}" in
    r2_a20)
      trace_root_override="${root_dir}/act_map_exp/trace_r2_a20"
      points3d_override="${root_dir}/act_map_exp/exp_data/warehouse_base_model_r2_a20/sparse/0/points3D.txt"
      ;;
    r1_a30)
      trace_root_override="${root_dir}/act_map_exp/trace_r1_a30"
      points3d_override="${root_dir}/act_map_exp/exp_data/warehouse_base_model_r1_a30/sparse/0/points3D.txt"
      ;;
    *)
      echo "Unknown dataset: ${dataset} (expected r2_a20 or r1_a30)" >&2
      exit 2
      ;;
  esac
fi

if [[ -n "${trace_root_override}" ]]; then
  export FOV_TRACE_ROOT="${trace_root_override}"
fi
if [[ -n "${points3d_override}" ]]; then
  export FOV_POINTS3D="${points3d_override}"
fi

export_cmds="$(
  python3 - <<'PY'
import os
import shlex
import sys
from pathlib import Path

try:
    import yaml  # type: ignore
except Exception as exc:
    sys.stderr.write("PyYAML is required to read optuna_fov_tune.yaml.\n")
    sys.exit(1)

raw_cfg = os.environ.get("FOV_TUNE_CONFIG", "")
if not raw_cfg:
    sys.stderr.write("FOV_TUNE_CONFIG is empty.\n")
    sys.exit(1)
config_path = Path(raw_cfg)
if not config_path.exists():
    sys.stderr.write(f"Config not found: {config_path}\n")
    sys.exit(1)

def load_yaml(path: Path):
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return {}
    return data

cfg = load_yaml(config_path)
best_path = cfg.get("best_params_yaml")
best_file = None
if best_path:
    cand = Path(best_path)
    if not cand.is_absolute():
        cand = config_path.parent / cand
    if cand.exists():
        best_file = cand

params = None
source = None
if best_file is not None:
    best_data = load_yaml(best_file)
    params = best_data.get("fixed_params")
    source = str(best_file)
else:
    params = cfg.get("initial_params") or cfg.get("fixed_params")
    source = f"{config_path} (initial_params/fixed_params)"
    sys.stderr.write(
        "Warning: best_params_yaml not found; falling back to "
        "initial_params/fixed_params in optuna_fov_tune.yaml.\n"
    )

if not isinstance(params, dict) or not params:
    sys.stderr.write(f"No params found in {source}.\n")
    sys.exit(1)

supported = {
    "max_iter",
    "ks",
    "base_step_scale",
    "min_step_deg",
    "max_step_deg",
    "step_norm_mode",
    "traj_jac_step",
    "fov_schedule",
}

filtered = {k: v for k, v in params.items() if k in supported and v is not None}
if not filtered:
    sys.stderr.write(f"No supported params found in {source}.\n")
    sys.exit(1)

def stringify(key, value):
    if key == "fov_schedule":
        if isinstance(value, (list, tuple)):
            return ",".join(str(v) for v in value)
    return str(value)

env_map = {
    "max_iter": "FOV_OPT_MAX_ITER",
    "ks": "FOV_OPT_KS",
    "base_step_scale": "FOV_OPT_BASE_STEP_SCALE",
    "min_step_deg": "FOV_OPT_MIN_STEP_DEG",
    "max_step_deg": "FOV_OPT_MAX_STEP_DEG",
    "step_norm_mode": "FOV_OPT_STEP_NORM_MODE",
    "traj_jac_step": "FOV_OPT_TRAJ_JAC_STEP",
    "fov_schedule": "FOV_OPT_FOV_SCHEDULE",
}

for key, env_name in env_map.items():
    if key not in filtered:
        continue
    value = stringify(key, filtered[key])
    print(f"export {env_name}={shlex.quote(value)}")
PY
)"

if [[ -z "${export_cmds}" ]]; then
  echo "No optimization parameters resolved from ${config}." >&2
  exit 1
fi

eval "${export_cmds}"

cmd_args=()
if [[ "${mode}" == "along" ]]; then
  cmd_args+=(--along-path)
elif [[ "${mode}" == "both" ]]; then
  cmd_args+=(--both)
fi
cmd_args+=("${views[@]}")

exec "${script}" "${cmd_args[@]}"
