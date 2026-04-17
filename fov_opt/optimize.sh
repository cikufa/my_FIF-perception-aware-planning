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
Usage: $(basename "$0") [--along-path] [--dataset NAME] [--trace-root PATH] [view1 view2 ...]
  --along-path  Run along-path optimization (default).
  --dataset     r2_a20, r1_a30, or both (sets trace root and points3d defaults).
  --trace-root  Override trace root directory.
  --points3d    Override points3D.txt path.
  --config      Override optuna_fov_tune.yaml.
  --summarize   Summarize quiver metrics after optimization (same params/config).
               Writes *_pose_metrics_progress_full.csv when debug logs are available.
  --summarize-variation NAME  Only summarize a specific variation (e.g., top).
  --summarize-no-plots         Skip PNG output from summarizer.

Reads best params from optuna results (best_params_yaml in optuna_fov_tune.yaml).
If missing, falls back to initial_params/fixed_params in optuna_fov_tune.yaml.
USAGE
}

mode="along"
views=()
dataset=""
trace_root_override=""
points3d_override=""
config_override=""
best_params_override=""
dataset_mode="single"
summarize=false
summarize_variation=""
summarize_no_plots=false
explicit_config=false
print_params=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --normal|--both)
      echo "Error: only optimized_path_yaw is supported now (use --along-path or no mode)." >&2
      exit 2
      ;;
    --along-path|--along_path)
      mode="along"
      ;;
    --dataset)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --dataset" >&2
        exit 2
      fi
      dataset="$2"
      shift
      ;;
    --trace-root|--trace_root)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --trace-root" >&2
        exit 2
      fi
      trace_root_override="$2"
      shift
      ;;
    --points3d|--points3d-file|--points3d_file)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --points3d" >&2
        exit 2
      fi
      points3d_override="$2"
      shift
      ;;
    --config|--tune-config|--tune_config)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --config" >&2
        exit 2
      fi
      config_override="$2"
      explicit_config=true
      shift
      ;;
    --summarize)
      summarize=true
      ;;
    --summarize-variation)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --summarize-variation" >&2
        exit 2
      fi
      summarize_variation="$2"
      shift
      ;;
    --summarize-no-plots)
      summarize_no_plots=true
      ;;
    --print-params)
      print_params=true
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

if [[ -n "${dataset}" ]]; then
  case "${dataset}" in
    r2_a20)
      trace_root_override="${root_dir}/act_map_exp/trace_r2_a20"
      points3d_override="${root_dir}/act_map_exp/exp_data/warehouse_base_model_r2_a20/sparse/0/points3D.txt"
      if [[ -z "${config_override}" && -z "${FOV_BEST_PARAMS_YAML:-}" ]]; then
        best_params_override="${root_dir}/fov_opt/optuna/optuna_best_params.yaml"
      fi
      ;;
    r1_a30)
      trace_root_override="${root_dir}/act_map_exp/trace_r1_a30"
      points3d_override="${root_dir}/act_map_exp/exp_data/warehouse_base_model_r1_a30/sparse/0/points3D.txt"
      if [[ -z "${config_override}" && -z "${FOV_BEST_PARAMS_YAML:-}" ]]; then
        best_params_override="${root_dir}/fov_opt/optuna/optuna_best_params.yaml"
      fi
      ;;
    both)
      dataset_mode="both"
      if [[ -z "${config_override}" && -z "${FOV_BEST_PARAMS_YAML:-}" ]]; then
        best_params_override="${root_dir}/fov_opt/optuna/optuna_best_params.yaml"
      fi
      ;;
    *)
      echo "Unknown dataset: ${dataset} (expected r2_a20, r1_a30, or both)" >&2
      exit 2
      ;;
  esac
fi

if [[ -n "${config_override}" ]]; then
  config="${config_override}"
fi
export FOV_TUNE_CONFIG="${config}"
if [[ "${explicit_config}" == "true" ]]; then
  export FOV_PREFER_CONFIG_PARAMS="1"
  if [[ -z "${FOV_BEST_PARAMS_YAML:-}" ]]; then
    best_params_override=""
  fi
fi
if [[ "${print_params}" == "true" ]]; then
  export FOV_PRINT_PARAMS="1"
fi
if [[ "${summarize}" == "true" && -z "${FOV_OPT_DEBUG_LOG:-}" && -z "${FOV_OPT_DEBUG_LOG_PATH:-}" ]]; then
  export FOV_OPT_DEBUG_LOG=1
fi
if [[ -z "${FOV_BEST_PARAMS_YAML:-}" && -n "${best_params_override}" ]]; then
  if [[ -f "${best_params_override}" ]]; then
    export FOV_BEST_PARAMS_YAML="${best_params_override}"
  else
    sys_best="${root_dir}/fov_opt/optuna/optuna_best_params.yaml"
    if [[ -f "${sys_best}" ]]; then
      export FOV_BEST_PARAMS_YAML="${sys_best}"
    fi
  fi
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
prefer_config = os.environ.get("FOV_PREFER_CONFIG_PARAMS", "").strip().lower() in (
    "1", "true", "yes", "y", "on"
)
best_override = ""
if not prefer_config:
    best_override = os.environ.get("FOV_BEST_PARAMS_YAML", "")
best_path = cfg.get("best_params_yaml")
best_file = None
if best_override:
    override_path = Path(best_override)
    if override_path.exists():
        best_file = override_path
    else:
        sys.stderr.write(f"Warning: FOV_BEST_PARAMS_YAML not found: {override_path}\n")
if best_path and best_file is None and not prefer_config:
    cand = Path(best_path)
    if not cand.is_absolute():
        cand = config_path.parent / cand
    if cand.exists() and best_file is None:
        best_file = cand

supported = {
    "max_iteration",
    "ks",
    "ks_transition_deg",
    "base_step_scale",
    "min_step_deg",
    "max_step_deg",
    "step_norm_mode",
    "trajectory_jacobian_step",
    "fov_schedule",
    # legacy keys
    "max_iter",
    "traj_jac_step",
}

def collect_supported(src):
    if not isinstance(src, dict):
        return {}
    return {k: v for k, v in src.items() if k in supported and v is not None}

def apply_aliases(params):
    if "max_iter" in params and "max_iteration" not in params:
        params["max_iteration"] = params["max_iter"]
    if "traj_jac_step" in params and "trajectory_jacobian_step" not in params:
        params["trajectory_jacobian_step"] = params["traj_jac_step"]
    params.pop("max_iter", None)
    params.pop("traj_jac_step", None)
    return params

params = None
source = None
if best_file is not None:
    best_data = load_yaml(best_file)
    params = apply_aliases(collect_supported(best_data.get("fixed_params")))
    source = str(best_file)
else:
    params = {}
    cfg_fixed = cfg.get("fixed_params")
    cfg_initial = cfg.get("initial_params")
    if isinstance(cfg_fixed, dict) and cfg_fixed:
        params.update(cfg_fixed)
    elif isinstance(cfg_initial, dict) and cfg_initial:
        params.update(cfg_initial)
    # Fill from top-level supported keys (if present).
    for key, value in collect_supported(cfg).items():
        params.setdefault(key, value)
    params = apply_aliases(params)
    source = f"{config_path} (config params)"
    if not params and best_path and not prefer_config:
        sys.stderr.write(
            "Warning: best_params_yaml not found; falling back to config params.\n"
        )

if not isinstance(params, dict) or not params:
    sys.stderr.write(f"No params found in {source}.\n")
    sys.exit(1)

filtered = {k: v for k, v in params.items() if k in supported and v is not None}
if not filtered:
    sys.stderr.write(f"No supported params found in {source}.\n")
    sys.exit(1)
if os.environ.get("FOV_PRINT_PARAMS", "").strip():
    sys.stderr.write(f"[optimize] params source: {source}\n")
    for key in sorted(filtered.keys()):
        sys.stderr.write(f"[optimize] {key}={filtered[key]}\n")

def stringify(key, value):
    if key == "fov_schedule":
        if isinstance(value, (list, tuple)):
            return ",".join(str(v) for v in value)
    return str(value)

env_map = {
    "max_iteration": "FOV_OPT_MAX_ITERATION",
    "ks": "FOV_OPT_KS",
    "ks_transition_deg": "FOV_OPT_KS_TRANSITION_DEG",
    "base_step_scale": "FOV_OPT_BASE_STEP_SCALE",
    "min_step_deg": "FOV_OPT_MIN_STEP_DEG",
    "max_step_deg": "FOV_OPT_MAX_STEP_DEG",
    "step_norm_mode": "FOV_OPT_STEP_NORM_MODE",
    "trajectory_jacobian_step": "FOV_OPT_TRAJECTORY_JACOBIAN_STEP",
    "fov_schedule": "FOV_OPT_FOV_SCHEDULE",
}

for key, env_name in env_map.items():
    if key not in filtered:
        continue
    value = stringify(key, filtered[key])
    print(f"export {env_name}={shlex.quote(value)}")

if cfg.get("ks_from_visibility"):
    print("export FOV_OPT_KS_FROM_VISIBILITY=1")
    if "ks_transition_deg" not in filtered and cfg.get("ks_transition_deg") is not None:
        print(f"export FOV_OPT_KS_TRANSITION_DEG={shlex.quote(str(cfg.get('ks_transition_deg')))}")
PY
)"

if [[ -z "${export_cmds}" ]]; then
  echo "No optimization parameters resolved from ${config}." >&2
  exit 1
fi

eval "${export_cmds}"

cmd_args=(--along-path)
cmd_args+=("${views[@]}")

default_trace_root="${root_dir}/act_map_exp/trace_r2_a20"
default_points3d_r2="${root_dir}/act_map_exp/exp_data/warehouse_base_model_r2_a20/sparse/0/points3D.txt"
default_points3d_r1="${root_dir}/act_map_exp/exp_data/warehouse_base_model_r1_a30/sparse/0/points3D.txt"

resolve_paths() {
  local ds="$1"
  local trace_root="${default_trace_root}"
  local points3d="${default_points3d_r2}"
  if [[ "${ds}" == "r1_a30" ]]; then
    trace_root="${root_dir}/act_map_exp/trace_r1_a30"
    points3d="${default_points3d_r1}"
  elif [[ "${ds}" == "r2_a20" ]]; then
    trace_root="${root_dir}/act_map_exp/trace_r2_a20"
    points3d="${default_points3d_r2}"
  fi
  if [[ -n "${trace_root_override}" ]]; then
    trace_root="${trace_root_override}"
  fi
  if [[ -n "${points3d_override}" ]]; then
    points3d="${points3d_override}"
  fi
  if [[ ! -f "${points3d}" && -f "${default_points3d_legacy}" ]]; then
    points3d="${default_points3d_legacy}"
  fi
  echo "${trace_root}|${points3d}"
}

summarize_quivers() {
  local trace_root="$1"
  local points3d="$2"
  local summary_script="${root_dir}/act_map_exp/scripts/summarize_quiver_metrics.py"
  if [[ ! -f "${summary_script}" ]]; then
    echo "Missing summarize script: ${summary_script}" >&2
    return 1
  fi
  if [[ ! -d "${trace_root}" ]]; then
    echo "Trace root not found for summarizer: ${trace_root}" >&2
    return 1
  fi
  if ! find "${trace_root}" -type f -name 'quivers*.txt' -print -quit | grep -q .; then
    echo "No quiver files found under ${trace_root}; skipping summarize." >&2
    return 0
  fi
  local base_args=(--root "${trace_root}" --points "${points3d}" --pattern "quivers_path_yaw.txt")
  if [[ "${summarize_no_plots}" == "true" ]]; then
    base_args+=(--no-plots)
  fi
  if [[ -n "${summarize_variation}" ]]; then
    python3 "${summary_script}" "${base_args[@]}" --variation "${summarize_variation}"
    return $?
  fi
  if [[ ${#views[@]} -gt 0 ]]; then
    for v in "${views[@]}"; do
      if ! find "${trace_root}/${v}" -type f -name 'quivers*.txt' -print -quit | grep -q .; then
        echo "No quiver files found for variation ${v} under ${trace_root}; skipping." >&2
        continue
      fi
      python3 "${summary_script}" "${base_args[@]}" --variation "${v}"
    done
    return 0
  fi
  python3 "${summary_script}" "${base_args[@]}"
}

if [[ "${dataset_mode}" == "both" ]]; then
  if [[ -n "${trace_root_override}" || -n "${points3d_override}" ]]; then
    echo "--dataset both cannot be combined with --trace-root or --points3d" >&2
    exit 2
  fi
  trace_root_r1="${root_dir}/act_map_exp/trace_r1_a30"
  points3d_r1="${default_points3d_r1}"
  trace_root_r2="${root_dir}/act_map_exp/trace_r2_a20"
  points3d_r2="${default_points3d_r2}"
  FOV_TRACE_ROOT="${trace_root_r1}" \
  FOV_POINTS3D="${points3d_r1}" \
    "${script}" "${cmd_args[@]}"
  FOV_TRACE_ROOT="${trace_root_r2}" \
  FOV_POINTS3D="${points3d_r2}" \
    "${script}" "${cmd_args[@]}"
  if [[ "${summarize}" == "true" ]]; then
    summarize_quivers "${trace_root_r1}" "${points3d_r1}"
    summarize_quivers "${trace_root_r2}" "${points3d_r2}"
  fi
  exit 0
fi

resolved="$(resolve_paths "${dataset}")"
trace_root="${resolved%%|*}"
points3d="${resolved#*|}"

FOV_TRACE_ROOT="${trace_root}" \
FOV_POINTS3D="${points3d}" \
  "${script}" "${cmd_args[@]}"

if [[ "${summarize}" == "true" ]]; then
  summarize_quivers "${trace_root}" "${points3d}"
fi
