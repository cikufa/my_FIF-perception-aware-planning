#!/usr/bin/env bash
# Trajectory-optimization analysis only. RRT uses fov_opt/analyze.sh unchanged.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analyzer="${root_dir}/fov_opt/analyze_all.py"
trace_xyz="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt_xyz"
trace_full="${root_dir}/act_map_exp/trace_trajopt_r1_a30/traj_opt"
wh_xyz="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/warehouse_all_xyz_only.yaml"
wh_full="${root_dir}/act_map_exp/params/quad_traj_opt/warehouse/warehouse_all.yaml"
base_rrt="${root_dir}/act_map_exp/params/quad_rrt/base_analysis_cfg.yaml"
base_to="${root_dir}/act_map_exp/params/quad_traj_opt/base_analysis_cfg.yaml"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--xyz] [--full] [--no-merge] [--pipeline-cfg FILE] [analyze_all.py args...]

  Default: traj_opt + warehouse_all.yaml, and pulls optimized TE/RE from
            traj_opt_xyz (same warehouse_* names). FoV outputs live only under xyz.

  --xyz:       analyze traj_opt_xyz only (no merge; none + optimized there).
  --full:      traj_opt only, single-tree pipeline; disables merge (same as --no-merge).
  --no-merge:  do not pass --merge-optimized-from (e.g. custom top_dir layout).
  --pipeline-cfg FILE: read fov_opt/trajopt_pipeline.yaml. Overrides top-dir,
               warehouse yaml, base ana cfg, merge-from, --include-trajs, and
               (optionally) filters analysis_cfg.yaml by variations in place
               (restored after the run).
  --map DATASET: dataset suffix (r2_a20 / r1_a30) -> overrides the pipeline
               YAML's top-level `dataset` key, driving trace/traj_opt_<DATASET>.
               Alias: --dataset. Implies --pipeline-cfg. Trajectories /
               variations come from the YAML, not from this flag.
  --occ {w_occ,wo_occ,path_yaw}: shorthand overriding the YAML optimized_subdir
               with optimized_<value>. Implies --pipeline-cfg.
  --optimized-subdir NAME: override optimized_subdir with an exact folder name
               (e.g. optimized_w_occ). Implies --pipeline-cfg.

Env: FOV_TRAJOPT_TOP_DIR, FOV_TRAJOPT_WAREHOUSE_YAML, FOV_TRAJOPT_BASE_ANA_CFG,
     FOV_TRAJOPT_MERGE_OPTIMIZED_FROM (default: traj_opt_xyz), FOV_TRAJOPT_NO_MERGE_OPTIMIZED=1
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_full}}"
wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_full}}"
base_ana="${FOV_TRAJOPT_BASE_ANA_CFG:-}"
if [[ -z "${base_ana}" ]]; then
  if [[ -f "${base_to}" ]]; then
    base_ana="${base_to}"
  else
    base_ana="${base_rrt}"
  fi
fi

use_xyz_only=false
no_merge=false
pipeline_cfg="${FOV_TRAJOPT_PIPELINE_CFG:-}"
optimized_subdir_override=""
dataset_override=""
extra_args=()

while [[ $# -gt 0 ]]; do
  case "${1:-}" in
    --xyz)
      top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_xyz}}"
      wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_xyz}}"
      use_xyz_only=true
      shift
      ;;
    --full)
      top_dir="${FOV_TRAJOPT_TOP_DIR:-${trace_full}}"
      wh_yaml="${FOV_TRAJOPT_WAREHOUSE_YAML:-${wh_full}}"
      no_merge=true
      shift
      ;;
    --no-merge)
      no_merge=true
      shift
      ;;
    --pipeline-cfg|--pipeline_cfg)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --pipeline-cfg" >&2
        exit 2
      fi
      pipeline_cfg="$2"
      shift 2
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
      shift 2
      ;;
    --optimized-subdir|--optimized_subdir)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --optimized-subdir" >&2
        exit 2
      fi
      optimized_subdir_override="$2"
      shift 2
      ;;
    --map|--dataset)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --map" >&2
        exit 2
      fi
      dataset_override="$2"
      shift 2
      ;;
    *)
      extra_args+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${pipeline_cfg}" ]] && [[ -n "${optimized_subdir_override}" || -n "${dataset_override}" ]]; then
  pipeline_cfg="${root_dir}/fov_opt/trajopt_pipeline.yaml"
fi

filter_backup=""
cleanup_filter() {
  if [[ -n "${filter_backup}" && -f "${filter_backup}" ]]; then
    python3 - <<'PY' "${filter_backup}"
import json, shutil, sys
from pathlib import Path
p = Path(sys.argv[1])
try:
    pairs = json.loads(p.read_text())
except Exception:
    sys.exit(0)
for backup, orig in pairs:
    b = Path(backup)
    if b.exists():
        shutil.move(str(b), orig)
p.unlink(missing_ok=True)
PY
  fi
}
trap cleanup_filter EXIT

if [[ -n "${pipeline_cfg}" ]]; then
  if [[ ! -f "${pipeline_cfg}" ]]; then
    echo "--pipeline-cfg file not found: ${pipeline_cfg}" >&2
    exit 1
  fi
  gen_out="$(PIPELINE_CFG="${pipeline_cfg}" REPO_ROOT="${root_dir}" \
    OPTIMIZED_SUBDIR_OVERRIDE="${optimized_subdir_override}" \
    DATASET_OVERRIDE="${dataset_override}" \
    python3 - <<'PY'
import os, sys, shlex, json, tempfile
from pathlib import Path
import yaml

cfg_path = Path(os.environ['PIPELINE_CFG']).resolve()
repo_root = Path(os.environ['REPO_ROOT']).resolve()
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
warehouse_dir  = _p('warehouse_params_dir', repo / 'act_map_exp/params/quad_traj_opt/warehouse')
base_ana_cfg   = _p('base_analysis_cfg',    repo / 'act_map_exp/params/quad_traj_opt/base_analysis_cfg.yaml')
generated_dir  = _p('generated_dir',  repo / 'fov_opt/.generated')
generated_dir.mkdir(parents=True, exist_ok=True)

trajs_all = cfg.get('trajectories_all') or []
vars_all  = cfg.get('variations_all')   or []

optimized_subdir_default = cfg.get('optimized_subdir') or 'optimized_path_yaw'
optimized_subdir_override = os.environ.get('OPTIMIZED_SUBDIR_OVERRIDE', '').strip()
optimized_subdir = optimized_subdir_override or optimized_subdir_default

section = cfg.get('analyze') or {}
mode = (section.get('mode') or 'merge').lower()  # merge | xyz_only | full
trajs = section.get('trajectories') or trajs_all
variations_filter = section.get('variations')  # None means keep all
extra_args = section.get('extra_args') or []

if mode == 'xyz_only':
    top_dir = trace_xyz
    merge_from = None
elif mode == 'full':
    top_dir = trace_variants
    merge_from = None
else:  # merge
    top_dir = trace_variants
    merge_from = trace_xyz

# Warehouse-all yaml filtered to selected trajectories. Absolute paths so the
# generator can live under fov_opt/.generated/ without breaking resolution.
planner_doc = {
    'all': {
        'base': {str(warehouse_dir / f'{t}_base.yaml'): t for t in trajs},
        'var':  [str(warehouse_dir / 'variations' / f'{v}.yaml') for v in vars_all],
    }
}
wh_yaml = generated_dir / 'warehouse_analyze.yaml'
with open(wh_yaml, 'w') as f:
    yaml.safe_dump(planner_doc, f, sort_keys=False)

# Variations filter: patch analysis_cfg.yaml in-place with a backup manifest.
backup_manifest = ''
if variations_filter:
    allowed_types = set(variations_filter)
    pairs = []
    for traj in trajs:
        cfg_file = top_dir / traj / 'analysis_cfg.yaml'
        if not cfg_file.exists():
            continue
        with open(cfg_file) as f:
            data = yaml.safe_load(f) or {}
        trajs_block = data.get('trajectories') or {}
        new_block = {}
        for key, meta in trajs_block.items():
            t = (meta or {}).get('type') if isinstance(meta, dict) else None
            # Always keep FoV-optimized folders so "ours" stays in the analysis.
            if t in ('optimized', 'optimized_path_yaw'):
                new_block[key] = meta
            elif t in allowed_types:
                new_block[key] = meta
        if new_block == trajs_block:
            continue
        backup = cfg_file.with_suffix('.yaml.pipeline_bak')
        if not backup.exists():
            cfg_file.rename(backup)
        with open(cfg_file, 'w') as f:
            data_new = dict(data)
            data_new['trajectories'] = new_block
            yaml.safe_dump(data_new, f, sort_keys=False)
        pairs.append([str(backup), str(cfg_file)])
    if pairs:
        manifest = Path(tempfile.mkstemp(prefix='trajopt_ana_filter_', suffix='.json')[1])
        manifest.write_text(json.dumps(pairs))
        backup_manifest = str(manifest)

include_trajs = ','.join(trajs) if trajs else ''
extra_str = ' '.join(shlex.quote(str(a)) for a in extra_args)
print(f'PIPE_TOP_DIR={shlex.quote(str(top_dir))}')
print(f'PIPE_WAREHOUSE_YAML={shlex.quote(str(wh_yaml))}')
print(f'PIPE_BASE_ANA_CFG={shlex.quote(str(base_ana_cfg))}')
print(f'PIPE_MERGE_FROM={shlex.quote(str(merge_from)) if merge_from else "__none__"}')
print(f'PIPE_INCLUDE_TRAJS={shlex.quote(include_trajs)}')
print(f'PIPE_EXTRA_ARGS={shlex.quote(extra_str)}')
print(f'PIPE_FILTER_BACKUP={shlex.quote(backup_manifest)}')
print(f'PIPE_OPTIMIZED_SUBDIR={shlex.quote(optimized_subdir)}')
PY
  )"
  eval "${gen_out}"
  top_dir="${PIPE_TOP_DIR:-${top_dir}}"
  wh_yaml="${PIPE_WAREHOUSE_YAML:-${wh_yaml}}"
  base_ana="${PIPE_BASE_ANA_CFG:-${base_ana}}"
  if [[ -n "${PIPE_FILTER_BACKUP:-}" ]]; then
    filter_backup="${PIPE_FILTER_BACKUP}"
  fi
  if [[ "${PIPE_MERGE_FROM:-}" == "__none__" ]]; then
    no_merge=true
    use_xyz_only=true  # prevents the default merge_from below
  else
    FOV_TRAJOPT_MERGE_OPTIMIZED_FROM="${PIPE_MERGE_FROM}"
    export FOV_TRAJOPT_MERGE_OPTIMIZED_FROM
  fi
  if [[ -n "${PIPE_INCLUDE_TRAJS:-}" ]]; then
    extra_args=(--include-trajs "${PIPE_INCLUDE_TRAJS}" ${extra_args[@]+"${extra_args[@]}"})
  fi
  if [[ -n "${PIPE_EXTRA_ARGS:-}" ]]; then
    # shellcheck disable=SC2206
    _pipe_extra=( ${PIPE_EXTRA_ARGS} )
    extra_args+=( ${_pipe_extra[@]+"${_pipe_extra[@]}"} )
  fi
  if [[ -n "${PIPE_OPTIMIZED_SUBDIR:-}" ]]; then
    # analyze_pose_errors.py walks these names under <traj>_*/; the chosen
    # subdir becomes the only one we aggregate, so w_occ / wo_occ don't
    # collide when both exist on disk. optimized_path_yaw stays in the list
    # so legacy runs still merge.
    if [[ "${PIPE_OPTIMIZED_SUBDIR}" == "optimized_path_yaw" ]]; then
      export FOV_OPTIMIZED_DIR_NAMES="optimized,optimized_path_yaw"
    else
      export FOV_OPTIMIZED_DIR_NAMES="${PIPE_OPTIMIZED_SUBDIR}"
    fi
  fi
fi

merge_arg=()
if [[ "${FOV_TRAJOPT_NO_MERGE_OPTIMIZED:-}" == "1" ]]; then
  no_merge=true
fi
if [[ "${use_xyz_only}" == "false" && "${no_merge}" == "false" ]]; then
  merge_from="${FOV_TRAJOPT_MERGE_OPTIMIZED_FROM:-${trace_xyz}}"
  merge_arg=(--merge-optimized-from "${merge_from}")
fi

python3 "${analyzer}" \
  --top-dir "${top_dir}" \
  --warehouse-all-yaml "${wh_yaml}" \
  --base-ana-cfg "${base_ana}" \
  ${merge_arg[@]+"${merge_arg[@]}"} \
  ${extra_args[@]+"${extra_args[@]}"}
