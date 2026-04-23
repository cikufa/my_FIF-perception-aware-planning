#!/usr/bin/env bash
# Trajectory-optimization registration only. RRT uses fov_opt/register.sh unchanged.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner="${root_dir}/act_map_exp/scripts/run_planner_exp.py"
# Planner experiment list (maps × variations). Default is quad_traj_opt warehouse_all.yaml — not quad_rrt.
# Override: FOV_TRAJOPT_REG_CFG
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
pipeline_cfg="${FOV_TRAJOPT_PIPELINE_CFG:-}"
pipeline_along_path_flag=""
optimized_subdir_override=""
dataset_override=""

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--full] [--variants|--optimized|--all] [--no-along-path]
                        [--pipeline-cfg FILE] [extra run_planner_exp args]

  Default (no --full):
    --variants   top_outdir = trace_trajopt_r1_a30/traj_opt (all planned trajs / info variants).
                 traj_opt_xyz is NOT used here (that tree is only for FoV optimization inputs).
    --optimized  Registers .../traj_opt_xyz/.../optimized (output of optimize_trajopt.sh).
    --all        Runs variants then optimized (each with the correct defaults yaml).

  --full: variants and optimized both use traj_opt + run_planner_defaults_fov_full.yaml
          (full-pose traj-opt tree; FoV optimized folders live under traj_opt).

  --pipeline-cfg FILE: read fov_opt/trajopt_pipeline.yaml (paths + subsets) and
          auto-generate the planner / defaults YAMLs under fov_opt/.generated/.
          Uses register_variants / register_optimized sections of that YAML.
  --map DATASET: dataset suffix used in trace/traj_opt_<DATASET>/ (e.g.
          --map r2_a20, --map r1_a30). Overrides the pipeline YAML's top-level
          `dataset` key. Alias: --dataset. Implies --pipeline-cfg.
          Trajectory names (warehouse_*) and variations (gp_det, ...) live in
          the YAML, not here.
  --occ {w_occ,wo_occ,path_yaw}: shorthand that overrides the YAML
          optimized_subdir with optimized_<value>. Implies --pipeline-cfg.
  --optimized-subdir NAME: override optimized_subdir with an exact folder name
          (e.g. optimized_w_occ). Implies --pipeline-cfg.

Default planner yaml: <repo>/act_map_exp/params/quad_traj_opt/warehouse/warehouse_all.yaml
  Override: FOV_TRAJOPT_REG_CFG
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
    --pipeline-cfg|--pipeline_cfg)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --pipeline-cfg" >&2
        exit 2
      fi
      pipeline_cfg="$2"
      shift
      ;;
    --occ)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --occ" >&2
        exit 2
      fi
      case "$2" in
        w_occ|wo_occ|path_yaw) optimized_subdir_override="optimized_$2" ;;
        optimized_*)           optimized_subdir_override="$2" ;;
        *) echo "Invalid --occ value: $2 (expected w_occ|wo_occ|path_yaw)" >&2; exit 2 ;;
      esac
      shift
      ;;
    --optimized-subdir|--optimized_subdir)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --optimized-subdir" >&2
        exit 2
      fi
      optimized_subdir_override="$2"
      shift
      ;;
    --map|--dataset)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --map" >&2
        exit 2
      fi
      dataset_override="$2"
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

# --occ / --optimized-subdir / --map imply --pipeline-cfg at the default location.
if [[ -z "${pipeline_cfg}" ]] && [[ -n "${optimized_subdir_override}" || -n "${dataset_override}" ]]; then
  pipeline_cfg="${root_dir}/fov_opt/trajopt_pipeline.yaml"
fi

if [[ -n "${pipeline_cfg}" ]]; then
  if [[ ! -f "${pipeline_cfg}" ]]; then
    echo "--pipeline-cfg file not found: ${pipeline_cfg}" >&2
    exit 1
  fi
  if [[ "${mode}" == "all" ]]; then
    echo "--pipeline-cfg is incompatible with --all; invoke with --variants and --optimized separately." >&2
    exit 2
  fi
  gen_out="$(PIPELINE_CFG="${pipeline_cfg}" REGISTER_MODE="${mode}" REPO_ROOT="${root_dir}" \
    OPTIMIZED_SUBDIR_OVERRIDE="${optimized_subdir_override}" \
    DATASET_OVERRIDE="${dataset_override}" \
    python3 - <<'PY'
import os, sys, shlex
from pathlib import Path
import yaml

cfg_path = Path(os.environ['PIPELINE_CFG']).resolve()
repo_root = Path(os.environ['REPO_ROOT']).resolve()
mode = os.environ['REGISTER_MODE']  # variants | optimized | all

with open(cfg_path) as f:
    cfg = yaml.safe_load(f) or {}

dataset = (os.environ.get('DATASET_OVERRIDE', '').strip()
           or cfg.get('dataset', 'r1_a30'))
paths = cfg.get('paths') or {}
def _p(key, default):
    v = paths.get(key)
    return Path(v).resolve() if v else Path(default).resolve()

repo = _p('repo_root', repo_root)
trace_variants = _p('trace_variants', repo / 'act_map_exp/trace' / f'traj_opt_{dataset}' / 'traj_opt')
trace_xyz      = _p('trace_xyz',      repo / 'act_map_exp/trace' / f'traj_opt_{dataset}' / 'traj_opt_xyz')
colmap_scripts = _p('colmap_scripts', repo / 'act_map_exp/colmap_scripts')
base_model     = _p('base_model',     repo / 'act_map_exp/exp_data' / f'warehouse_base_model_{dataset}')
warehouse_dir  = _p('warehouse_params_dir', repo / 'act_map_exp/params/quad_traj_opt/warehouse')
generated_dir  = _p('generated_dir',  repo / 'fov_opt/.generated')
generated_dir.mkdir(parents=True, exist_ok=True)

trajs_all = cfg.get('trajectories_all') or []
vars_all  = cfg.get('variations_all')   or []

optimized_subdir_default = cfg.get('optimized_subdir') or 'optimized_path_yaw'
optimized_subdir_override = os.environ.get('OPTIMIZED_SUBDIR_OVERRIDE', '').strip()
optimized_subdir = optimized_subdir_override or optimized_subdir_default

# Section
if mode == 'optimized':
    section = cfg.get('register_optimized') or {}
else:
    section = cfg.get('register_variants') or {}
trajs = section.get('trajectories') or trajs_all
variations = section.get('variations') or vars_all
extra_args = section.get('extra_args') or []

# Build planner (warehouse-subset) yaml. Use absolute paths because
# run_planner_exp.py resolves bases/variations relative to the planner
# YAML's own directory, and the generated file lives under fov_opt/.generated/.
planner_doc = {
    'all': {
        'base': {str(warehouse_dir / f'{t}_base.yaml'): t for t in trajs},
        'var':  [str(warehouse_dir / 'variations' / f'{v}.yaml') for v in variations],
    }
}
planner_yaml = generated_dir / f'warehouse_{mode}.yaml'
with open(planner_yaml, 'w') as f:
    yaml.safe_dump(planner_doc, f, sort_keys=False)

# Build defaults yaml
if mode == 'optimized':
    opt_vars = section.get('variations') or ['none']
    # Compose optimized_dirs as <traj>/<traj>_<var>/<optimized_subdir>.
    # Keys are "<traj>" when var is 'none' (to match run_planner_exp.py exp_nm);
    # for other variations, the combined key lets _resolve_opt_dir() match
    # exp_nm via the positional fallback only when trajectories_all[i] == traj.
    optimized_dirs = {}
    for t in trajs:
        for var in opt_vars:
            leaf = f'{t}_{var}'
            candidate = trace_xyz / t / leaf / optimized_subdir
            if not candidate.exists():
                # Skip silently; the user's YAML may list variations that have
                # no corresponding optimized output (only _none is produced by
                # optimize_trajopt.sh in the current tree).
                continue
            key = t if var == 'none' else leaf
            optimized_dirs[key] = str(candidate)
    defaults_doc = {
        'top_outdir': str(trace_xyz),
        'colmap_script_dir': str(colmap_scripts),
        'base_model': str(base_model),
        'optimized_dirs': optimized_dirs,
    }
    defaults_yaml = generated_dir / 'defaults_optimized.yaml'
else:
    defaults_doc = {
        'top_outdir': str(trace_variants),
        'colmap_script_dir': str(colmap_scripts),
        'base_model': str(base_model),
    }
    defaults_yaml = generated_dir / 'defaults_variants.yaml'
with open(defaults_yaml, 'w') as f:
    yaml.safe_dump(defaults_doc, f, sort_keys=False)

along_path = 'true' if bool(section.get('along_path', False)) else 'false'
extra_str = ' '.join(shlex.quote(str(a)) for a in extra_args)
print(f'PIPE_PLANNER_YAML={shlex.quote(str(planner_yaml))}')
print(f'PIPE_DEFAULTS_YAML={shlex.quote(str(defaults_yaml))}')
print(f'PIPE_ALONG_PATH={along_path}')
print(f'PIPE_EXTRA_ARGS={shlex.quote(extra_str)}')
print(f'PIPE_OPTIMIZED_SUBDIR={shlex.quote(optimized_subdir)}')
PY
  )"
  eval "${gen_out}"
  if [[ -n "${PIPE_PLANNER_YAML:-}" ]]; then
    config="${PIPE_PLANNER_YAML}"
  fi
  if [[ -n "${PIPE_DEFAULTS_YAML:-}" ]]; then
    if [[ "${mode}" == "optimized" ]]; then
      defaults_optimized="${PIPE_DEFAULTS_YAML}"
    elif [[ "${mode}" == "variants" ]]; then
      defaults_variants="${PIPE_DEFAULTS_YAML}"
    else
      defaults_variants="${PIPE_DEFAULTS_YAML}"
      defaults_optimized="${PIPE_DEFAULTS_YAML}"
    fi
  fi
  if [[ "${mode}" != "optimized" && -n "${PIPE_ALONG_PATH:-}" ]]; then
    if [[ "${PIPE_ALONG_PATH}" == "true" ]]; then
      with_along_path=true
    else
      with_along_path=false
    fi
  fi
  if [[ -n "${PIPE_EXTRA_ARGS:-}" ]]; then
    # shellcheck disable=SC2206
    _pipe_extra=( ${PIPE_EXTRA_ARGS} )
    extra_args+=( "${_pipe_extra[@]}" )
  fi
  if [[ -n "${PIPE_OPTIMIZED_SUBDIR:-}" ]]; then
    # Make the chosen subdir recognized by run_planner_exp.py (safety net;
    # absolute paths in optimized_dirs already resolve directly).
    export FOV_OPTIMIZED_DIR_NAMES="optimized,optimized_path_yaw,${PIPE_OPTIMIZED_SUBDIR}"
  fi
fi

if [[ ! -f "$config" ]]; then
  echo "Planner config not found: ${config}" >&2
  exit 1
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
  local cmd=(python3 "$planner" "$config" --defaults_yaml "$defaults_file" --mode reg --skip_optimized)
  if [[ "${with_along_path}" == "true" ]]; then
    cmd+=(--along_path)
  fi
  cmd+=("${default_max_reg[@]}")
  cmd+=("${extra_args[@]}")
  "${cmd[@]}"
}

run_optimized() {
  local defaults_file="$1"
  local cmd=(python3 "$planner" "$config" --defaults_yaml "$defaults_file" --mode reg --only_optimized)
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
