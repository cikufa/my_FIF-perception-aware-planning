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
resume_n_trials=""
resume_total_trials=""

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--tune] [--along-path] [--dataset NAME] [extra optuna args]
  --tune       Run tuning mode.
  --along-path Force along_path=true.
  --dataset    r2_a20, r1_a30, or both (selects outputs/paths). Default: r2_a20.
  --workers N  Run N Optuna workers (multi-process, shared storage).
  --storage S  Override Optuna storage URL (e.g. sqlite:////path/optuna.db).
  --study-name NAME Override study name (shared across workers).
  --vis-weight W     Add visibility improvement term (minimization uses -W * vis_improvement).
  --vis-metric NAME  Visibility metric from quiver metrics (vis30_count or vis30_score).
  --vis-improvement MODE  delta or ratio (last-first or (last-first)/first).
  --vis-pattern NAME  Quiver metrics filename pattern (default: quivers_path_yaw_metrics.txt).
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
      echo "Error: only optimized_path_yaw is supported now (use --along-path or no mode)." >&2
      exit 2
      ;;
    --dataset)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --dataset" >&2
        exit 2
      fi
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

if [[ -z "${dataset}" ]]; then
  dataset="r2_a20"
fi

case "${dataset}" in
  r2_a20|r1_a30|both)
    ;;
  *)
    echo "Unknown dataset: ${dataset} (expected r2_a20, r1_a30, or both)" >&2
    exit 2
    ;;
esac

temp_cfg="/tmp/optuna_fov_tune_${dataset}.yaml"
BASE_CONFIG="${config}" ROOT_DIR="${root_dir}" DATASET="${dataset}" OUT_CONFIG="${temp_cfg}" \
    python3 - <<'PY'
import os
from pathlib import Path

try:
    import yaml  # type: ignore
except Exception:
    raise SystemExit("PyYAML is required to build dataset config.")

base_config = Path(os.environ["BASE_CONFIG"]).resolve()
root_dir = Path(os.environ["ROOT_DIR"]).resolve()
dataset = os.environ["DATASET"]
out_config = Path(os.environ["OUT_CONFIG"]).resolve()

def load_yaml(path: Path):
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return {}
    return data

cfg = load_yaml(base_config)

warehouse_cfg = load_yaml(root_dir / "act_map_exp" / "params" / "quad_rrt" / "warehouse" / "warehouse_all.yaml")
base_map = warehouse_cfg.get("all_cfg", {}).get("base", {})
views = [v for v in base_map.values() if isinstance(v, str)]

trace_r1 = root_dir / "act_map_exp" / "trace_r1_a30"
trace_r2 = root_dir / "act_map_exp" / "trace_r2_a20"
trace_both = root_dir / "act_map_exp" / "trace"
optuna_dir = root_dir / "fov_opt" / "optuna"
optuna_dir.mkdir(parents=True, exist_ok=True)

def pose_files(trace_root: Path, path_yaw: bool):
    out = []
    for view in views:
        out.append(str(trace_root / view / f"{view}_none" / "optimized_path_yaw" / "pose_errors.txt"))
    return out

def opt_cmd(ds: str):
    return f"./run_fov_opt_rrt_none.sh --along-path --dataset {ds}"

def reg_cmd(ds: str, along: bool):
    defaults = root_dir / "act_map_exp" / "params" / "quad_rrt" / "warehouse" / f"run_planner_defaults_{ds}.yaml"
    cmd = (
        f"python3 run_planner_exp.py {root_dir}/act_map_exp/params/quad_rrt/warehouse/warehouse_all.yaml "
        f"--defaults_yaml {defaults} --only_optimized --fail_on_error"
    )
    cmd += " --along_path"
    return cmd

if dataset == "r1_a30":
    roots = [trace_r1]
    if not cfg.get("vis_trace_root"):
        cfg["vis_trace_root"] = [str(trace_r1)]
    cfg["run_cmd_optimization"] = opt_cmd("r1_a30")
    cfg["run_cmd_registration_along_path"] = reg_cmd("r1_a30", True)
    cfg["run_cmd_registration_normal"] = reg_cmd("r1_a30", False)
    storage_dir = optuna_dir
elif dataset == "r2_a20":
    roots = [trace_r2]
    if not cfg.get("vis_trace_root"):
        cfg["vis_trace_root"] = [str(trace_r2)]
    cfg["run_cmd_optimization"] = opt_cmd("r2_a20")
    cfg["run_cmd_registration_along_path"] = reg_cmd("r2_a20", True)
    cfg["run_cmd_registration_normal"] = reg_cmd("r2_a20", False)
    storage_dir = optuna_dir
else:
    roots = [trace_r1, trace_r2]
    if not cfg.get("vis_trace_root"):
        cfg["vis_trace_root"] = [str(trace_r1), str(trace_r2)]
    cfg["run_cmd_optimization"] = f"{opt_cmd('r1_a30')} && {opt_cmd('r2_a20')}"
    cfg["run_cmd_registration_along_path"] = f"{reg_cmd('r1_a30', True)} && {reg_cmd('r2_a20', True)}"
    cfg["run_cmd_registration_normal"] = f"{reg_cmd('r1_a30', False)} && {reg_cmd('r2_a20', False)}"
    storage_dir = optuna_dir
    cfg["study_name"] = "fov_optuna_both"

cfg["pose_errors_files_along_path"] = []
cfg["pose_errors_files_normal"] = []
for root in roots:
    cfg["pose_errors_files_along_path"].extend(pose_files(root, True))
    cfg["pose_errors_files_normal"].extend(pose_files(root, False))

suffix = "both" if dataset == "both" else dataset
cfg["storage"] = f"sqlite:////{storage_dir}/optuna_{suffix}.db"
cfg["results_jsonl"] = f"{storage_dir}/optuna_trials_{suffix}.jsonl"
cfg["results_csv"] = f"{storage_dir}/optuna_trials_{suffix}.csv"
cfg["best_params_yaml"] = f"{storage_dir}/optuna_best_params.yaml"
cfg["pareto_params_yaml"] = f"{storage_dir}/optuna_pareto_params.yaml"

out_config.parent.mkdir(parents=True, exist_ok=True)
out_config.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
PY
config="${temp_cfg}"

cmd=(python3 "$script" --config "$config")
if [[ -n "$mode" ]]; then
  cmd+=(--mode "$mode")
fi
if [[ "$along" == "along" ]]; then
  cmd+=(--along-path)
elif [[ "$along" == "normal" ]]; then
  cmd+=(--normal)
fi

# Default to 10 registration images in tune mode unless caller overrides.
has_max_reg_trial=false
for arg in "${extra_args[@]}"; do
  case "$arg" in
    --max-reg-images-per-trial|--max_reg_images_per_trial|--max-reg-images-per-trial=*|--max_reg_images_per_trial=*)
      has_max_reg_trial=true
      break
      ;;
  esac
done
if [[ "$mode" == "tune" && "$has_max_reg_trial" == "false" ]]; then
  extra_args+=(--max-reg-images-per-trial 10)
fi
cmd+=("${extra_args[@]}")

# Resolve resume settings (use --n-trials total and existing storage to compute remaining).
# CLI overrides in extra_args take precedence over YAML; support both --flag value and --flag=value.
parse_override() {
  local key="$1"
  local -n out="$2"
  local i arg
  for ((i=0; i<${#extra_args[@]}; i++)); do
    arg="${extra_args[$i]}"
    if [[ "$arg" == "--${key}" ]]; then
      if [[ $((i+1)) -lt ${#extra_args[@]} ]]; then
        out="${extra_args[$((i+1))]}"
      fi
    elif [[ "$arg" == "--${key}="* ]]; then
      out="${arg#--${key}=}"
    fi
  done
}

n_trials_override=""
storage_override_cli="${storage_override}"
study_override_cli="${study_override}"
parse_override "n-trials" n_trials_override
parse_override "storage" storage_override_cli
parse_override "study-name" study_override_cli

resume_info="$(
  N_TRIALS_OVERRIDE="${n_trials_override}" \
  STORAGE_OVERRIDE="${storage_override_cli}" \
  STUDY_OVERRIDE="${study_override_cli}" \
  CONFIG_PATH="${config}" \
  python3 - <<'PY' || true
import os
from pathlib import Path

try:
    import yaml  # type: ignore
except Exception:
    print("")
    raise SystemExit(0)

try:
    import optuna  # type: ignore
except Exception:
    print("")
    raise SystemExit(0)

config_path = Path(os.environ.get("CONFIG_PATH", "")).resolve()
if not config_path.exists():
    print("")
    raise SystemExit(0)

cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
if not isinstance(cfg, dict):
    cfg = {}

n_trials_override = os.environ.get("N_TRIALS_OVERRIDE", "").strip()
n_trials = int(n_trials_override) if n_trials_override else int(cfg.get("n_trials") or 0)
if n_trials <= 0:
    print("")
    raise SystemExit(0)

storage_override = os.environ.get("STORAGE_OVERRIDE", "").strip()
storage = storage_override or str(cfg.get("storage") or "").strip()
study_override = os.environ.get("STUDY_OVERRIDE", "").strip()
study_name = study_override or cfg.get("study_name")

if not storage:
    print(f"{n_trials} {n_trials}")
    raise SystemExit(0)

if storage.startswith("sqlite:///"):
    path = storage.replace("sqlite:///", "", 1)
    if not Path(path).exists():
        print(f"{n_trials} {n_trials}")
        raise SystemExit(0)

try:
    study = optuna.load_study(study_name=study_name, storage=storage)
except Exception:
    print(f"{n_trials} {n_trials}")
    raise SystemExit(0)

from optuna.trial import TrialState
trials = study.get_trials(deepcopy=False)
done = sum(1 for t in trials if t.state != TrialState.RUNNING)
remaining = n_trials - done
if remaining < 0:
    remaining = 0
print(f"{remaining} {n_trials}")
PY
)"

if [[ -n "$resume_info" ]]; then
  read -r resume_n_trials resume_total_trials <<<"$resume_info"
  if [[ -n "${resume_n_trials}" && -n "${resume_total_trials}" ]]; then
    if [[ "${resume_n_trials}" -le 0 ]]; then
      echo "All ${resume_total_trials} trials already completed; nothing to resume."
      exit 0
    fi
    cmd+=(--n-trials "${resume_n_trials}")
    echo "Resuming Optuna: ${resume_n_trials} trials remaining of ${resume_total_trials}."
  fi
fi

if [[ "$workers" -le 1 ]]; then
  if [[ -n "$storage_override" ]]; then
    cmd+=(--storage "$storage_override")
  fi
  if [[ -n "$study_override" ]]; then
    cmd+=(--study-name "$study_override")
  fi
  exec "${cmd[@]}"
fi

default_storage="sqlite:////home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/fov_opt/optuna/optuna_r2_a20.db"
if [[ "${dataset}" == "r1_a30" ]]; then
  default_storage="sqlite:////home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/fov_opt/optuna/optuna_r1_a30.db"
elif [[ "${dataset}" == "both" ]]; then
  default_storage="sqlite:////home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/fov_opt/optuna/optuna_both.db"
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
