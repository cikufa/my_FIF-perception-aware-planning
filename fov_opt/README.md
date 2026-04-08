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
  - It optimizes the path-yaw variant only and writes outputs under
    `<view>_none/optimized_path_yaw`. It does **not** generate the along_path
    baseline folder.
  - Each optimization run writes `optimization_time_sec.txt` inside its output
    folder (e.g., `<view>_none/optimized_path_yaw/optimization_time_sec.txt`).
  - `--summarize` emits additional per-pose CSVs under
    `<view>_none/optimized_path_yaw/quiver_analysis`, including
    `*_pose_metrics_progress_full.csv` when debug logs are available.

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
  - `optimize_trajopt.sh` -> forwards to `optimize.sh` with `--trace-root` / `--points3d` for `trace_trajopt_r1_a30/traj_opt_xyz` by default (FoV on xyz-only `*_none`; `--full` uses `traj_opt`).  
  - `register_trajopt.sh` -> `warehouse_all.yaml` + `run_planner_defaults_reg_variants.yaml` for **`--variants`** (`top_outdir` = `traj_opt`), and `run_planner_defaults_reg_optimized_xyz.yaml` for **`--optimized`** (`optimized_path_yaw` under `traj_opt_xyz`). **`--full`** uses `run_planner_defaults_fov_full.yaml` for both (everything under `traj_opt`).  
  - `analyze_trajopt.sh` -> default `traj_opt` + `warehouse_all.yaml`; add **`--xyz`** for `traj_opt_xyz` + `warehouse_all_xyz_only.yaml` (FoV outputs). Uses `quad_traj_opt/base_analysis_cfg.yaml` when present.

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
- **`traj_opt_xyz`**: xyz-only `none` runs — used **only** as FoV optimization input (`optimize_trajopt.sh` default) and for **`register_trajopt.sh --optimized`** (`run_planner_defaults_reg_optimized_xyz.yaml`).
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

`./optimize_trajopt.sh --along-path`

`./register_trajopt.sh --optimized`

`./analyze_trajopt.sh`

`./analyze_trajopt.sh --xyz`

`./register_trajopt.sh --all` runs variants then optimized with the correct defaults each time.

Full single-tree pipeline (`traj_opt` only): add **`--full`** to **`optimize_trajopt.sh`** and **`register_trajopt.sh`** (see `run_planner_defaults_fov_full.yaml`). Env overrides: **`FOV_TRAJOPT_REG_DEFAULTS_VARIANTS`**, **`FOV_TRAJOPT_REG_DEFAULTS_OPTIMIZED`**, **`FOV_TRAJOPT_REG_DEFAULTS`** (with `--full`).

### End-to-end pipeline (traj opt + FoV + UE + TE/RE)

1. **Planning (outside these scripts)** saves stamped poses under e.g. `.../warehouse_top/warehouse_top_none/` (`stamped_Twc.txt`, UE convention files, etc.).

2. **`optimize_trajopt.sh` / `optimize.sh --trace-root ...`** runs `manifold_test_trajectory` on each `*_none` folder: warm-starts from **yaw along path** (`along_path/stamped_Twc_path_yaw.txt`, generated from positions when missing), refines **camera rotations** for FoV/visibility, writes `optimized_path_yaw/` (e.g. `stamped_Twc_path_yaw.txt`, `stamped_Twc_ue_path_yaw.txt`, quivers).

3. **`register_trajopt.sh`** calls `run_planner_exp.py`, which for each variation:
   - Builds **along_path** from `stamped_Twc.txt` via `generate_path_yaw.py` (path-aligned yaw + UE pose files).
   - **Renders** images in Unreal via `colmap_scripts/my_render_ue.py` using `stamped_Twc_ue_path_yaw.txt` (or optimized `*_path_yaw` / `optimized_stamped_Twc_ue_path_yaw.txt` where applicable).
   - **Registers** images to COLMAP (`register_images_to_model.py`).
   - **Computes TE/RE** (`calculate_pose_errors.py`).

4. So for the **along_path** baseline you get: stamped poses → yaw along path → UE render → registration → TE/RE. For **`optimized_path_yaw`**, the same render/register/eval steps use the **FoV-optimized** rotations instead of the initial path yaw.

5. **`analyze_trajopt.sh`** runs `analyze_pose_errors.py` (default **`traj_opt`**; **`--xyz`** for **`traj_opt_xyz`** FoV outputs).

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
