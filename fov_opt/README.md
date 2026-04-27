# FoV Optimization Helpers

This folder collects FoV optimization and tuning entrypoints without duplicating
pipeline logic. The heavy lifting stays in `act_map_exp/scripts`.

## Scripts

- `optimize.sh` -> calls `act_map_exp/scripts/run_fov_opt_rrt_none.sh`
  - Example: `./optimize.sh --along-path` (default)
  - Dataset shortcut (RRT only): `./optimize.sh --both --dataset r2_a20`, `--dataset r1_a30`, or `--dataset both`
  - Override trace root directly: `./optimize.sh --both --trace-root /path/to/trace`
  - Use a specific params yaml: `./optimize.sh --along-path --dataset r2_a20 --config /path/to/params.yaml`
    - When `--config` is provided, params are read from that YAML (`fixed_params` -> `initial_params` -> top-level supported keys).
    - When `--config` is omitted, params are read from `best_params_yaml` in `optuna_fov_tune.yaml` (fallback to `initial_params` / `fixed_params`).
  - Print resolved params and source: `./optimize.sh --print-params ...`
  - Summarize quiver metrics after optimization: `./optimize.sh --summarize ...`
    - `--summarize-variation top` limits to a single variation.
    - `--summarize-no-plots` skips PNGs.
  - Default views: all subdirs under `act_map_exp/trace_r2_a20` that contain
    `<view>_none` (override with explicit view names or `FOV_TRACE_ROOT`).
  - It optimizes the FoV orientation only and writes outputs under
    `<view>_none/<output-dir-name>`. The default output directory name is
    `optimized`; use `--output-dir-name` to change it.
  - Each optimization run writes `optimization_time_sec.txt` inside its output
    folder (e.g., `<view>_none/optimized_w_occ/optimization_time_sec.txt`).
  - `--summarize` emits additional per-pose CSVs under
    `<view>_none/<output-dir-name>/quiver_analysis`, including
    `*_pose_metrics_progress_full.csv` when debug logs are available.
  - After `analyze_pose_errors.py --multiple`, **`analysis_outputs/finite/`** includes **`pipeline_runtime*.tex`**, **`.txt`**, and **`.csv`** (and the same under **`fov_optimization_runtime*`**): **rows** = method variants (No Info., GP/Quad/PC, **Ours**); **columns** = mean traj-opt time (s) from `ceres_summary.yaml` as `custom_solve_time - custom_logger_time` (same as `analyze_traj_opt.py` / Table V–style Ceres timing), averaged over analyzed trajectories under `top_dir` for Fisher variants, and mean Ceres time for the xyz-only `*_none` run on **`--merge_optimized_from`** (or `top_dir` if unset) for **Ours**; second column is **FoV** manifold time (mean of `optimization_time_sec.txt` under `*_none/optimized_path_yaw/`) **for Ours only**, **0** for other rows.

- `register.sh` -> registration wrapper for `run_planner_exp.py`.
  - Variants always include along_path baseline (it generates along_path poses):  
    `./register.sh --variants`
  - Along_path baseline only (no other variants):  
    `./register.sh --variants --only_along_path`
  - Optimized only (optimized path-yaw): `./register.sh --optimized --along-path`
  - All (non-optimized + optimized): `./register.sh --all`
  - `optimized_path_yaw` is the **only** optimized output that is registered.
  - By default, registration uses **all frames** (it injects `--max_reg_images 0`
    unless you pass a `--max_reg_images` override).
  - Dataset shortcut (RRT only): `./register.sh --optimized --dataset r2_a20`, `--dataset r1_a30`, or `--dataset both`
  - Override defaults yaml directly: `./register.sh --optimized --defaults-yaml /path/to/run_planner_defaults.yaml`

- `tune.sh` -> tuning wrapper for `optuna_fov_tune.py`.
  - Tune normal: `./tune.sh --tune --normal`
  - Tune along-path: `./tune.sh --tune --along-path`
  - Fixed runs are now `./optimize.sh` + `./register.sh --optimized`.
  - Dataset shortcut: `./tune.sh --tune --dataset r2_a20`, `--dataset r1_a30`, or `--dataset both`
  - Resume: re-running with the same `--n-trials` resumes remaining trials from storage.
  - Override config with `--config /path/to/optuna_fov_tune.yaml`.

- `analyze_all.py` -> analyze all variations under the trace root (RRT traces).
  - Example: `./analyze_all.py --plt-min-ratio 0.2 --plt-max-ratio 1.0`
  - Dataset shortcut: `./analyze_all.py --dataset r2_a20` or `--dataset r1_a30`
  - Optional: `--warehouse-all-yaml` to choose which warehouse yaml supplies `--include` views (default: `quad_rrt/warehouse/warehouse_all.yaml`)
  - Override trace root with `--top-dir /path/to/trace_r2_a20`.
  - Paper figures: each mode folder (`analysis_outputs/finite`, `analysis_outputs/original`,
    `analysis_outputs/penalized`) now includes `paper_figs/` with publication-ready plots:
    - `overall_mean_std_errors*.png`: average of TE/RE means for each variation (for quick visual comparison).
    - `overall_max_errors*.png`: max TE/RE bars with paper labels.
    - `overall_te_re_avg_norm*.png`: three stacked bar charts (TE mean, RE mean, normalized avg error).
      The normalized average uses **Option 2**:  
      `avg_norm = 0.5 * (TE / TE_ref + RE / RE_ref)`  
      where `TE_ref` and `RE_ref` default to the **no info** baseline (`type=none`) if present,
      otherwise they fall back to the median of finite means across variations. This yields a unitless
      composite score that can be compared across methods without mixing meters and degrees directly.
    - `registration_failure_table*.png`: per-config registration failure rates (NaN ratio in pose errors),
      with `X` indicating no valid samples for that method in the config.

- `analyze.sh` -> shell wrapper for `analyze_all.py` (RRT).
  - Example: `./analyze.sh --plt-min-ratio 0.2 --plt-max-ratio 1.0`

- **Trajectory optimization (separate entrypoints; RRT scripts above are unchanged)**  
  - `optimize_trajopt.sh` -> resolve traj-opt FoV inputs from `--map r2_a20|r1_a30`; defaults to `trace/traj_opt_<map>/traj_opt_xyz` (FoV on xyz-only `*_none`; `--full` uses `traj_opt`). It also auto-resolves `points3D.txt`, `FOV_OPT_ESDF_PATH`, and `FOV_MANIFOLD_BIN` unless already overridden in the environment.  
  - `register_trajopt.sh` -> `warehouse_all.yaml` + `run_planner_defaults_reg_variants.yaml` for **`--variants`** (`top_outdir` = `traj_opt`), and `run_planner_defaults_reg_optimized_xyz.yaml` for **`--optimized`** (`optimized_path_yaw` under `traj_opt_xyz`). **`--full`** uses `run_planner_defaults_fov_full.yaml` for both (everything under `traj_opt`).  
  - `analyze_trajopt.sh` -> default `traj_opt` + `warehouse_all.yaml`, and **`--merge-optimized-from traj_opt_xyz`** so **`optimized_path_yaw`** appears in plots (FoV registration lives under xyz). **`--xyz`** analyzes xyz only (no merge). **`--no-merge`** / **`register_trajopt.sh --full`**-style single tree: use `--full` or `--no-merge`. Uses `quad_traj_opt/base_analysis_cfg.yaml` when present.

- `act_map_exp/scripts/visualize_quiver_progress.py` -> RViz animation of FoV
  optimization iterations from `quivers*.txt`.
  - Example (path-yaw top):  
    `python3 ../act_map_exp/scripts/visualize_quiver_progress.py --view top --variant none --path-yaw --rate 2.0`
  - Example (direct folder):  
    `python3 ../act_map_exp/scripts/visualize_quiver_progress.py --run-dir ../act_map_exp/trace_r2_a20/top/top_none/optimized_path_yaw --rate 2.0`
  - ROS2 note: run the script directly after `source /opt/ros/humble/setup.bash`
    and use `rviz2` with a MarkerArray display on `/stamped_pose_viz`.

- `act_map_exp/scripts/summarize_quiver_metrics.py` -> recompute per-iteration
  alignment/visibility metrics from `quivers*.txt`.
  - Example: `python3 ../act_map_exp/scripts/summarize_quiver_metrics.py --root ../act_map_exp/trace_r2_a20 --full`
  - `--full` adds jacobian/debug columns from `optimization_debug.csv` when available.
  - `--debug-log /path/to/optimization_debug.csv` forces a specific debug log.
  - If a matching `quivers*_metrics.txt` file exists (and shape matches),
    visibility count/score are read from it to match the optimizer’s own metrics.


## Current Run Commands

These are the current wrapper commands for the FoV optimization flows in this
checkout.

- Best params default:
  `optimize.sh`, `optimize_rrt.sh`, and `optimize_trajopt.sh` now default to
  `fov_opt/optuna/optuna_best_params.yaml`.
- Ray occlusion:
  `--with-ray-occlusion` enables ESDF ray-based filtering.
  `--without-ray-occlusion` disables it.
- Saved feature lists:
  if `FOV_OPT_DUMP_VISIBLE_FEATURES=1`, each run writes
  `visible_features_per_iteration.txt`, which `fov_pose_story_scene.launch`
  reads directly.

### RRT

Run all RRT views for `r1_a30` with ray occlusion, writing to `optimized_w_occ`:

```bash
cd /home/shekoufeh/fov/FIF_ws/src/rpg_information_field
export FOV_OPT_DUMP_VISIBLE_FEATURES=1
./fov_opt/optimize_rrt.sh --map r1_a30 --with-ray-occlusion --output-dir-name optimized_w_occ
```

Run all RRT views for `r1_a30` without ray occlusion, writing to `optimized_wo_occ`:

```bash
cd /home/shekoufeh/fov/FIF_ws/src/rpg_information_field
export FOV_OPT_DUMP_VISIBLE_FEATURES=1
./fov_opt/optimize_rrt.sh --map r1_a30 --without-ray-occlusion --output-dir-name optimized_wo_occ
```

Run a single RRT view:

```bash
./fov_opt/optimize_rrt.sh --map r1_a30 --with-ray-occlusion --output-dir-name optimized_w_occ bottom
./fov_opt/optimize_rrt.sh --map r1_a30 --without-ray-occlusion --output-dir-name optimized_wo_occ bottom
```

Compare-mode run in one command:

```bash
./fov_opt/optimize_rrt.sh --map r1_a30 --compare-ray-occlusion --output-dir-name optimized
```

This writes:
- `optimized_with_ray_occlusion`
- `optimized_without_ray_occlusion`

### Trajectory Optimization

Run all traj-opt xyz-only `*_none` views for `r2_a20` with ray occlusion:

```bash
cd /home/shekoufeh/fov/FIF_ws/src/rpg_information_field
export FOV_OPT_DUMP_VISIBLE_FEATURES=1
./fov_opt/optimize_trajopt.sh --map r2_a20 --with-ray-occlusion --output-dir-name optimized_w_occ
```

Run all traj-opt xyz-only `*_none` views for `r2_a20` without ray occlusion:

```bash
cd /home/shekoufeh/fov/FIF_ws/src/rpg_information_field
export FOV_OPT_DUMP_VISIBLE_FEATURES=1
./fov_opt/optimize_trajopt.sh --map r2_a20 --without-ray-occlusion --output-dir-name optimized_wo_occ
```

Run a single traj-opt view:

```bash
./fov_opt/optimize_trajopt.sh --map r2_a20 --with-ray-occlusion --output-dir-name optimized_w_occ warehouse_diagonal
./fov_opt/optimize_trajopt.sh --map r2_a20 --without-ray-occlusion --output-dir-name optimized_wo_occ warehouse_diagonal
```

Compare-mode run in one command:

```bash
./fov_opt/optimize_trajopt.sh --map r2_a20 --compare-ray-occlusion --output-dir-name optimized
```

This writes:
- `optimized_with_ray_occlusion`
- `optimized_without_ray_occlusion`

### Direct `optimize.sh`

Use this when you want to point directly at a trace root.

RRT `r1_a30` with ray occlusion:

```bash
./fov_opt/optimize.sh \
  --trace-root /home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/trace/trace_rrt_r1_a30 \
  --points3d /home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_base_model_r1_a30/sparse/0/points3D.txt \
  --with-ray-occlusion \
  --output-dir-name optimized_w_occ
```

RRT `r1_a30` without ray occlusion:

```bash
./fov_opt/optimize.sh \
  --trace-root /home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/trace/trace_rrt_r1_a30 \
  --points3d /home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/exp_data/warehouse_base_model_r1_a30/sparse/0/points3D.txt \
  --without-ray-occlusion \
  --output-dir-name optimized_wo_occ
```

### Visualize Saved Quivers and Saved Visible Features in RViz

`fov_pose_story_scene.launch` now expects a quiver file with a matching saved
feature sidecar next to it. For current FoV outputs, that means:
- `per_iteration_quivers.txt`
- `visible_features_per_iteration.txt`

Example:

```bash
source /opt/ros/noetic/setup.bash
source /home/shekoufeh/fov/FIF_ws/devel/setup.bash

roslaunch act_map_exp fov_pose_story_scene.launch \
  quivers:=/home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map_exp/trace/traj_opt_r2_a20/traj_opt_xyz/warehouse_diagonal/warehouse_diagonal_none/optimized_wo_occ/per_iteration_quivers.txt \
  pose_index:=7
```

Notes:
- `pose_index` is 0-based.
- The launch no longer takes `fov_deg`; it only reads saved feature lists.
- The visualizer overlays all iterations for the selected pose, colored from
  red to green across optimization progress.


## Config

- Tuning config: `optuna_fov_tune.yaml`
- Result outputs (now saved under `fov_opt/optuna`):
  - `fov_opt/optuna/optuna_trials_r2_a20.jsonl`
  - `fov_opt/optuna/optuna_best_params.yaml`
  - `fov_opt/optuna/optuna_pareto_params.yaml`

## Optimization Parameters (optuna_fov_tune.yaml)

The optimizer parameters below are forwarded to the C++ binary
`My_FoV_Optimization/Manifold_cpp/build/manifold_test_trajectory` via
environment variables (see `trajectory_test.cpp` and
`trajectory_optimizer_copy.h`).
Names in YAML mirror the C++ member names where possible. Keys with `*_deg`
are specified in degrees and converted to radians before they populate the
corresponding `*_rad` members in C++.

**Core optimizer parameters**

| Key | Meaning | Notes |
| --- | --- | --- |
| `max_iteration` | Max optimization iterations. | Maps to `max_iteration` in `trajectory_optimizer_copy.h` (exported as `FOV_OPT_MAX_ITERATION`). |
| `ks` | Visibility sigmoid sharpness (higher = steeper cutoff). | Used in `w = -ks * (u - cos(alpha))` inside the visibility sigmoid. |
| `ks_transition_deg` | Target visibility transition width (degrees). | Used to compute `ks` when `ks_from_visibility: true` (see below). Also passed to the optimizer for per-iteration ks if enabled. |
| `base_step_scale` | Global step size scale. | Multiplies normalized Jacobian before clamping. |
| `min_step_deg` | Minimum per-iteration rotation step (deg). | Clamps the step after scaling; internally decays with iteration (maps to `min_step_rad`). |
| `max_step_deg` | Maximum per-iteration rotation step (deg). | Same clamp as above, with a separate decay (maps to `max_step_rad`). |
| `step_norm_mode` | Step normalization mode. | `points` uses number of valid points; `jacobian` uses avg Jacobian norm. |
| `trajectory_jacobian_step` | Smoothness weight for trajectory Jacobian. | Scales the finite-difference Jacobian before adding to FoV gradient (`FOV_OPT_TRAJECTORY_JACOBIAN_STEP`). |
| `fov_schedule` | Comma-separated FoV **half-angles** (deg) for staged optimization. | Each stage length is `max_iteration / len(schedule)`; stage `i` uses `schedule[i]` (alpha), mapped to `fov_schedule_rad` in C++. |
| `log_jacobian` | Enable optimizer Jacobian debug logging. | Sets `FOV_OPT_LOG_JACOBIAN`. |
| `FOV_OPT_DEBUG_LOG` | Enable per-iter debug CSV logging. | **Env-only** (not a YAML key). Writes `optimization_debug.csv` next to outputs; used by `summarize_quiver_metrics.py --full`. |
| `FOV_OPT_DEBUG_LOG_PATH` | Optional debug CSV path. | **Env-only** (not a YAML key). Overrides default debug log location. |
| `warm_start` | Start from a previous trajectory file. | Uses `FOV_OPT_WARM_START`. |
| `warm_start_file` | Optional warm-start file path. | Uses `FOV_OPT_WARM_START_FILE`; defaults to previous output if present. |

**Tuning controls and ranges (Optuna)**

| Key | Meaning | Notes |
| --- | --- | --- |
| `tune_*` | Toggle whether Optuna samples a parameter. | If false (or overridden by `fixed_params`), the value is fixed. Examples: `tune_max_iteration`, `tune_trajectory_jacobian_step`. |
| `max_iteration` | Max optimization iterations. | If you want to **force** 30 every trial, put `max_iteration: 30` in `fixed_params` (or set `tune_max_iteration: true` with `max_iteration_range: [30, 30]`). |
| `*_range` | Numeric range for a parameter. | Used by Optuna when the matching `tune_*` is enabled (e.g., `max_iteration_range`, `trajectory_jacobian_step_range`, `min_step_deg_range`). |
| `*_options` | Categorical options for a parameter. | Used for `step_norm_mode` and `fov_schedule`. |
| `base_step_scale_log` / `ks_log` | Use log-uniform sampling. | Applies to `base_step_scale` / `ks`. |
| `fixed_params` | Hard overrides for optimizer params. | Any key here disables tuning for that parameter. |
| `initial_params` | Initial trial values. | Used when `enqueue_initial: true` to seed Optuna. |
| `ks_from_visibility` | Derive `ks` from FoV schedule. | Uses last value in `fov_schedule` as `alpha` and `ks_transition_deg` to compute `ks ≈ 2.94 / (Δα * sin α)` (approx. 5%→95% sigmoid width). When enabled, the optimizer also recomputes ks per iteration based on the current `alpha` stage. |
| `ks_visibility_range_multipliers` | Expansion range around derived `ks`. | If `tune_ks: true`, Optuna samples `ks` in `[ks*lo, ks*hi]`. |
| `ks_mode_options` | Discrete ks choices for Optuna. | If set, Optuna samples `ks_mode` from this list (e.g., `const15`, `const20`, `const71`, `map`). `map` uses `ks_from_visibility` with `ks_transition_deg`. |
| `stat` | Metric aggregation. | `min`, `max`, `avg`, or `std` over pose errors. |
| `pose_error_mode` | How to treat NaNs in pose errors. | `finite` drops NaNs, `original` replaces NaNs with `1.2*hist_max_*`, `penalized` replaces NaNs with `max_trans_e_m/max_rot_e_deg` (fallback to `hist_max_*`). Matches `analyze.sh` modes. |
| `base_analysis_cfg` | Base analysis config for penalty defaults. | Used to read `hist_max_trans_e` / `hist_max_rot_e` when computing `original`/`penalized` objectives. Defaults to `act_map_exp/params/quad_rrt/base_analysis_cfg.yaml`. |
| `te_weight`, `re_weight` | Objective weights. | Objective is `te_weight * TE + re_weight * RE` in single-objective mode. |
| `multi_objective` | Enable 2D objective. | Minimizes TE and RE separately (Pareto front). |
| `max_reg_images_per_trial` | Cap registration **and rendering** images per trial. | `run_planner_exp.py` subsamples pose lists to this count. |
| `render_*_tune` | Rendering overrides used during tuning. | Passed to `run_planner_exp.py` (width/height/FoV/sleep). |
| `n_trials`, `timeout`, `n_jobs` | Optuna run controls. | `n_jobs` is Optuna’s internal parallelism. |
| `storage`, `study_name`, `seed` | Optuna storage/config. | `storage` is a DB URL (sqlite by default). |
| `results_jsonl`, `results_csv` | Trial output files. | Written after the study completes. |
| `best_params_yaml`, `pareto_params_yaml` | Output summaries. | Best params or Pareto front params. |

## Trace Roots

Use these paths when you want to pass explicit arguments.

r2_a20:
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/trace_r2_a20`
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r2_a20.yaml`

r1_a30:
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/trace_r1_a30`
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r1_a30.yaml`

Trajectory optimization (params under `act_map_exp/params/quad_traj_opt/warehouse/`):

- **`traj_opt`**: full planned trajectories (all info variants) — used for **`register_trajopt.sh --variants`** (`run_planner_defaults_reg_variants.yaml`).
- **`traj_opt_xyz`**: xyz-only **`none`** baselines only (no gp/quad/pc variants on disk) — FoV input (`optimize_trajopt.sh` default) and **`register_trajopt.sh --optimized`**. Reflected in `warehouse_all_xyz_only.yaml` (single `variations/none.yaml` entry).
- **`--full`** on `optimize_trajopt.sh` / `register_trajopt.sh`: single tree `traj_opt` + `run_planner_defaults_fov_full.yaml`.

## Typical Runs

r2_a20:
`./optimize.sh --along-path --dataset r2_a20`
`./register.sh --all --dataset r2_a20`
`./analyze_all.py --dataset r2_a20`

r1_a30:
`./optimize.sh --along-path --dataset r1_a30`
`./register.sh --all --dataset r1_a30`
`./analyze_all.py --dataset r1_a30`

both:
`./optimize.sh --along-path --dataset both`
`./register.sh --all --dataset both`

Trajectory optimization — use **`optimize_trajopt.sh` / `register_trajopt.sh` / `analyze_trajopt.sh`** (RRT scripts stay separate).

Recommended order: **`--variants`** uses **`traj_opt`**; **`optimize_trajopt.sh`** uses **`traj_opt_xyz`** `*_none`; **`--optimized`** registers **`traj_opt_xyz`** `optimized_path_yaw`.

`./register_trajopt.sh --variants`

`./optimize_trajopt.sh --map r2_a20`

`./register_trajopt.sh --optimized`

`./analyze_trajopt.sh` — variants from **`traj_opt`**, optimized series merged from **`traj_opt_xyz`**.

`./analyze_trajopt.sh --xyz` — xyz tree only (none + optimized there).

`./register_trajopt.sh --all` runs variants then optimized with the correct defaults each time.

Full single-tree pipeline (`traj_opt` only): add **`--full`** to **`optimize_trajopt.sh`** and **`register_trajopt.sh`** (see `run_planner_defaults_fov_full.yaml`). Env overrides: **`FOV_TRAJOPT_REG_DEFAULTS_VARIANTS`**, **`FOV_TRAJOPT_REG_DEFAULTS_OPTIMIZED`**, **`FOV_TRAJOPT_REG_DEFAULTS`** (with `--full`).

## Command Cookbook (copy/paste)

Use this section as a quick reminder of what to run and why.

### RRT pipeline

1) **Run FoV optimization on `_none` trajectories** (writes `optimized_path_yaw/`):

```bash
./optimize.sh --along-path --dataset r2_a20
```

- What it does: runs `manifold_test_trajectory` on each `<view>_none`, optimizing rotations only, and saves `optimization_time_sec.txt` + optimized quivers in `<view>_none/optimized_path_yaw/`.

2) **Register and evaluate localization** (variants + optimized):

```bash
./register.sh --all --dataset r2_a20
```

- What it does: uses `run_planner_exp.py` to render UE images, run COLMAP registration, and compute `pose_errors.txt` for all variants and `optimized_path_yaw`.

3) **Analyze and compare methods**:

```bash
./analyze.sh --dataset r2_a20
```

- What it does: runs `analyze_all.py` -> `analyze_pose_errors.py --multiple`, producing `analysis_outputs/{finite,original,penalized}` with plots/tables.

4) **Custom trace root example**:

```bash
./register.sh --all --top_outdir /path/to/trace_r2_a20 --defaults-yaml /path/to/run_planner_defaults.yaml
./analyze.sh --top-dir /path/to/trace_r2_a20
```

- Use when your experiment lives outside default `act_map_exp/trace_r2_a20`.

---

### Trajectory optimization pipeline (split trees: `traj_opt` + `traj_opt_xyz`)

Your intended flow is:
- `traj_opt`: all planned variants (`none`, `gp_*`, `quad_*`, `pc_*`)
- `traj_opt_xyz`: FoV optimization input/output for `*_none` only (`optimized_path_yaw`)

1) **Register/evaluate planned variants in `traj_opt`**:

```bash
./register_trajopt.sh --variants
```

- What it does: renders/registers/evaluates all planned traj-opt variants from `traj_opt`.

2) **Run FoV optimization on xyz-none trajectories**:

```bash
./optimize_trajopt.sh --map r2_a20
```

- What it does: writes FoV-optimized results under `traj_opt_xyz/<view>/<view>_none/optimized_path_yaw/`.

3) **Register/evaluate optimized FoV outputs from `traj_opt_xyz`**:

```bash
./register_trajopt.sh --optimized
```

- What it does: localizes only `optimized_path_yaw` outputs listed in `run_planner_defaults_reg_optimized_xyz.yaml`.

4) **Analyze merged comparison** (variants from `traj_opt` + optimized from `traj_opt_xyz`):

```bash
./analyze_trajopt.sh
```

- What it does: analyzes `traj_opt` and injects optimized TE/RE from `traj_opt_xyz` via `--merge-optimized-from`.

5) **Analyze xyz tree only** (debug/check):

```bash
./analyze_trajopt.sh --xyz
```

- What it does: no merge, only `traj_opt_xyz` tree.

6) **Single-tree full mode** (`traj_opt` only):

```bash
./optimize_trajopt.sh --map r2_a20 --full
./register_trajopt.sh --full --all
./analyze_trajopt.sh --full
```

- What it does: keeps variants + optimized in one tree (`traj_opt`), disables split-tree merge logic.

---

### YAML-driven trajectory-optimization pipeline (`trajopt_pipeline.yaml`)

`register_trajopt.sh` and `analyze_trajopt.sh` can be driven directly from a
single master config, **`fov_opt/trajopt_pipeline.yaml`**, which is the
recommended entrypoint for the w_occ / wo_occ comparison (rendering + eval go
directly under `optimized_w_occ/` or `optimized_wo_occ/`, not
`optimized_path_yaw/`).

Relevant CLI flags (both scripts share them):

| Flag | Effect |
| --- | --- |
| `--pipeline-cfg FILE` | Parse this YAML and auto-generate planner + defaults YAMLs under `fov_opt/.generated/`. Implied by `--map` / `--occ` / `--optimized-subdir`. |
| `--map {r2_a20\|r1_a30}` (alias `--dataset`) | Override the `dataset` key in the YAML. Controls which `traj_opt_<map>` trace tree is used. Trajectory / variation selection stays in the YAML. |
| `--occ {w_occ\|wo_occ\|path_yaw}` | Picks which optimized subdir to work with. `w_occ`/`wo_occ` target `optimized_w_occ` / `optimized_wo_occ`; `path_yaw` keeps the legacy `optimized_path_yaw`. |
| `--optimized-subdir NAME` | Fully custom optimized-subdir name (bypasses `--occ`). For `analyze_trajopt.sh`, accepts a **comma-separated list** (e.g. `optimized_w_occ,optimized_depthmap`) to plot each variant as its own series in the same figures. |

Default trace layout (resolved from `--map <DATASET>`):

```
act_map_exp/trace/traj_opt_<DATASET>/
├── traj_opt/                                            # Fisher baseline + variants
│   └── warehouse_<traj>/<traj>_<var>/{pose_errors.txt, ...}
└── traj_opt_xyz/                                        # FoV-optimized (ours)
    └── warehouse_<traj>/<traj>_none/
        └── optimized_<subdir>/{pose_errors.txt, eval/, ...}
```

`<subdir>` is one of `optimized_w_occ`, `optimized_wo_occ`, `optimized_depthmap`,
or the legacy `optimized_path_yaw`. Both `register_trajopt.sh` and
`analyze_trajopt.sh` derive these defaults automatically; no `paths.*`
overrides are needed for the standard layout.

Minimal runs on `r1_a30`:

```bash
# 1. Register Fisher baseline (none, gp_*, quad_*, pc_*) under traj_opt/:
./fov_opt/register_trajopt.sh --variants --map r1_a30

# 2. Register FoV-optimized variants under traj_opt_xyz/ (one variant per call):
./fov_opt/register_trajopt.sh --optimized --map r1_a30 --optimized-subdir optimized_w_occ
./fov_opt/register_trajopt.sh --optimized --map r1_a30 --optimized-subdir optimized_depthmap

# 3a. Analyze: baselines vs single ours:
./fov_opt/analyze_trajopt.sh --map r1_a30 --optimized-subdir optimized_w_occ
./fov_opt/analyze_trajopt.sh --map r1_a30 --optimized-subdir optimized_depthmap

# 3b. Analyze: baselines vs both ours variants together (each as its own series):
./fov_opt/analyze_trajopt.sh --map r1_a30 \
    --optimized-subdir optimized_w_occ,optimized_depthmap

# Sanity check on FoV-only (xyz tree only, no Fisher comparison):
./fov_opt/analyze_trajopt.sh --map r1_a30 --occ w_occ   # set analyze.mode: xyz_only in YAML
```

For `r2_a20`, replace `--map r1_a30` with `--map r2_a20`.

Multi-variant analysis:

- The analyzer extension keys each entry in `FOV_OPTIMIZED_DIR_NAMES` (set
  automatically from `--optimized-subdir`) under its own dictionary key when
  more than one name is supplied, so `optimized_w_occ` and
  `optimized_depthmap` appear as **distinct series** in violin/histogram/CSV
  outputs instead of being collapsed into a single `optimized_path_yaw` line.
- Single-name behavior is unchanged for backward compatibility — anything
  matching the legacy `optimized_path_yaw` collapsed series still works.
- Per-variant default labels: `optimized_w_occ → "ours (w occ)"`,
  `optimized_wo_occ → "ours (wo occ)"`, `optimized_depthmap → "ours (depthmap)"`.
  Override by editing `labels` / `colors` / `linestyles` in
  `act_map_exp/params/quad_traj_opt/base_analysis_cfg.yaml`.
- Comparison CSV (`overall_error_compare_vs_optimized.csv`) uses the **first**
  active optimized key as the reference; the remaining optimized variants
  appear in the table as additional rows.

Selecting a subset of trajectories / variations is YAML-only; edit the
`register_variants` / `register_optimized` / `analyze` sections of
`trajopt_pipeline.yaml`:

```yaml
register_optimized:
  trajectories: [warehouse_mid, warehouse_top]
  variations:   [none]

analyze:
  mode:         merge        # merge | xyz_only | full
  trajectories: [warehouse_mid, warehouse_top]
  variations:   [none, gp_det, quad_det]
```

`trajectories: null` (or a missing list) means "use every entry in
`trajectories_all`". `variations: null` means "keep every variation present
on disk".

Implementation notes:

- The scripts parse the YAML with an inline Python heredoc and emit runtime
  planner/defaults YAMLs into **`fov_opt/.generated/`** (gitignored).
- Downstream Python (`run_planner_exp.py`, `analyze_pose_errors.py`) reads
  the environment variable **`FOV_OPTIMIZED_DIR_NAMES`** that the scripts
  export, so `optimized_w_occ` and `optimized_wo_occ` stay in separate
  analyses and never mix.
- `analyze_trajopt.sh` temporarily filters each trajectory's
  `analysis_cfg.yaml` down to the variants requested in the YAML and
  restores it via a shell trap when the run exits (or fails).
- Matplotlib TeX cache: if your `~/.cache/matplotlib` is not writable, run
  with `MPLCONFIGDIR=fov_opt/.mplcache ./fov_opt/analyze_trajopt.sh ...`
  (the path is gitignored).

### Registration-failure thresholds

Pose-error statistics treat each rendered frame as a **success/failure**
event using fixed thresholds in
`act_map_exp/params/quad_traj_opt/base_analysis_cfg.yaml`:

```yaml
hist_max_trans_e: 1.0    # translation cap τ_t, metres
hist_max_rot_e:   10     # rotation cap τ_r, degrees
```

A frame is a failure iff `e_t > τ_t` OR `e_r > τ_r`; failed frames are
masked with NaN before any aggregation. The analyzer then reports three
complementary summaries under `analysis_outputs/{finite,original,penalized}/`:

- **`finite/`** — stats over the success set only, paired with the per-method
  failure rate `ρ = (#NaN) / N`. Use for "how accurate when it works".
- **`penalized/`** — NaNs replaced by the threshold before aggregation, so the
  reported mean is  
  `ē_pen = (1 − ρ)·ē_fin + ρ·τ`.  
  This is the fair single-number metric: a method cannot lower it by failing
  more often. Recommended as the headline accuracy number alongside `ρ`.
- **`original/`** — NaNs rendered at `1.2·τ` for violin plots only, so the
  failure mass shows as a distinct band above the success distribution. Never
  used for numerical comparison.

Note: violin axis limits in `analyze_pose_errors.py` are currently fixed at
`VIOLIN_YLIM_TE = (0, 1.0)` and `VIOLIN_YLIM_RE = (0, 10.0)`; when using
`original/` mode the `1.2·τ` spike sits just above the axis and gets clipped.
Bump these constants if you want the failure band visible in the plot.

### End-to-end pipeline (traj opt + FoV + UE + TE/RE)

1. **Planning (outside these scripts)** saves stamped poses under e.g. `.../warehouse_top/warehouse_top_none/` (`stamped_Twc.txt`, UE convention files, etc.).

2. **`optimize_trajopt.sh` / `optimize.sh --trace-root ...`** runs `manifold_test_trajectory` on each `*_none` folder: warm-starts from **yaw along path** (`along_path/stamped_Twc_path_yaw.txt`, generated from positions when missing), refines **camera rotations** for FoV/visibility, writes `optimized_path_yaw/` (e.g. `stamped_Twc_path_yaw.txt`, `stamped_Twc_ue_path_yaw.txt`, quivers).

3. **`register_trajopt.sh`** calls `run_planner_exp.py`, which for each variation:
   - Builds **along_path** from `stamped_Twc.txt` via `generate_path_yaw.py` (path-aligned yaw + UE pose files).
   - **Renders** images in Unreal via `colmap_scripts/my_render_ue.py` using `stamped_Twc_ue_path_yaw.txt` (or optimized `*_path_yaw` / `optimized_stamped_Twc_ue_path_yaw.txt` where applicable).
   - **Registers** images to COLMAP (`register_images_to_model.py`).
   - **Computes TE/RE** (`calculate_pose_errors.py`).

4. So for the **along_path** baseline you get: stamped poses → yaw along path → UE render → registration → TE/RE. For **`optimized_path_yaw`**, the same render/register/eval steps use the **FoV-optimized** rotations instead of the initial path yaw.

5. **`analyze_trajopt.sh`** runs `analyze_pose_errors.py` on **`traj_opt`** and merges **`optimized_path_yaw`** pose errors from **`traj_opt_xyz`** (`analyze_pose_errors.py --merge_optimized_from`). Use **`--xyz`** to analyze only the xyz tree; **`--no-merge`** if optimized lives under **`traj_opt`** (`register_trajopt.sh --full`).

### Traj opt vs RRT: frames, formats, and NaNs in `pose_errors.txt`

- **Planner outputs** use the same convention as RRT: `stamped_Twc.txt` / `stamped_Twc_ue.txt` from `PlannerBase` (`quad_traj_opt_impl.h` / `quad_rrt_impl.h`), same `TwcToUEPose`, same `T_BC` pattern in base YAMLs. Traj-opt maps use names like `warehouse_left` instead of `left`; registration must use **`warehouse_base_model_r1_a30/warehouse_left_none_sparse`** (etc.), not the short-name sparse dirs, but the **pipeline is the same** `run_planner_exp.py` path as RRT.
- **NaNs in `pose_errors.txt`** (from `calculate_pose_errors.py`) mean that image **did not end up in the COLMAP sparse model** after `register_images_to_model.py` (path mismatch or **too few inliers**). It is **not** a silent frame mix-up: matching uses `rel_path` from `warehouse_*_none_sparse/../images` to the render folder, consistent with how models were built.
- **Extra NaNs in analysis** can come from `analyze_pose_errors.py` when TE/RE exceed `max_trans_e_m` / `max_rot_e_deg` in `analysis_cfg.yaml` (treated as invalid for “finite” stats).
- **Why more NaNs than RRT on disk**: traj-opt often has **denser time sampling** (`integral_cost_sample_dt`, many more poses per run), so more frames are hard for SIFT/COLMAP; try lowering `--min_num_inliers`, checking `registration_stats*.csv`, or subsampling with `run_planner_exp.py --max_reg_images`.

## Notes

- Registration pipeline lives in `act_map_exp/scripts/run_planner_exp.py`.
- `run_planner_exp.py --max_reg_images` now also limits rendering by subsampling the pose list.
- Along_path baseline poses are generated during `./register.sh --variants --along-path`.
- `register.sh` defaults to **all frames** unless you pass `--max_reg_images`.
- `optimize.sh` reads best params from `best_params_yaml` in
  `optuna_fov_tune.yaml` (falls back to `initial_params`/`fixed_params` if the
  best file is missing). If `--config` is provided, params are resolved from
  that YAML and `best_params_yaml` is ignored.
- `optuna_fov_tune.yaml` points `workdir` to `act_map_exp/scripts`, so the
  relative `run_fov_opt_rrt_none.sh` command still works.
- The optimizer binary `manifold_test_trajectory` is built from
  `My_FoV_Optimization/Manifold_cpp/trajectory_test.cpp`, which currently
  includes `trajectory_optimizer_copy.h`.
- Legacy env vars `FOV_OPT_MAX_ITER` and `FOV_OPT_TRAJ_JAC_STEP` are still
  accepted by the C++ binary for compatibility, but the canonical names are
  `FOV_OPT_MAX_ITERATION` and `FOV_OPT_TRAJECTORY_JACOBIAN_STEP`.
- If you enable `ks_from_visibility`, rebuild `manifold_test_trajectory` so the
  per-iteration ks logic is picked up.
- `optuna_fov_tune.py` enforces categorical options for existing studies.
  If you change `fov_schedule_options` or `step_norm_mode_options`, start a new
  study or keep the same options/order.
