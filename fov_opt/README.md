# FoV Optimization Helpers

This folder collects FoV optimization and tuning entrypoints without duplicating
pipeline logic. The heavy lifting stays in `act_map_exp/scripts`.

## Scripts

- `optimize.sh` -> calls `act_map_exp/scripts/run_fov_opt_rrt_none.sh`
  - Example: `./optimize.sh --both`
  - Dataset shortcut: `./optimize.sh --both --dataset r2_a20` or `--dataset r1_a30`
  - Override trace root directly: `./optimize.sh --both --trace-root /path/to/trace`
  - Default views: all subdirs under `act_map_exp/trace_r2_a20` that contain
    `<view>_none` (override with explicit view names or `FOV_TRACE_ROOT`).
  - When `--along-path` or `--both` is used, it generates along-path poses and
    saves them under `<view>_none/along_path`.
  - Each optimization run writes `optimization_time_sec.txt` inside its output
    folder (e.g., `<view>_none/optimized/optimization_time_sec.txt`).

- `register.sh` -> registration wrapper for `run_planner_exp.py`.
  - Non-optimized variants only: `./register.sh --variants`
  - Optimized only (normal): `./register.sh --optimized --normal`
  - Optimized only (optimized path-yaw): `./register.sh --optimized --along-path`
  - All (non-optimized + optimized): `./register.sh --all`
  - When optimized registration runs, it also registers `<view>_none/along_path`
    if present. It appears in analysis as `<view>_none_along_path`.
  - Dataset shortcut: `./register.sh --optimized --dataset r2_a20` or `--dataset r1_a30`
  - Override defaults yaml directly: `./register.sh --optimized --defaults-yaml /path/to/run_planner_defaults.yaml`

- `tune.sh` -> tuning wrapper for `optuna_fov_tune.py`.
  - Tune normal: `./tune.sh --tune --normal`
  - Tune along-path: `./tune.sh --tune --along-path`
  - Fixed runs are now `./optimize.sh` + `./register.sh --optimized`.
  - Dataset shortcut: `./tune.sh --tune --dataset r2_a20` or `--dataset r1_a30`
  - Override config with `--config /path/to/optuna_fov_tune.yaml`.

- `analyze_all.py` -> analyze all variations under the trace root.
  - Example: `./analyze_all.py --plt-min-ratio 0.2 --plt-max-ratio 1.0`
  - Dataset shortcut: `./analyze_all.py --dataset r2_a20` or `--dataset r1_a30`
  - Override trace root with `--top-dir /path/to/trace_r2_a20`.

- `analyze.sh` -> shell wrapper for `analyze_all.py`.
  - Example: `./analyze.sh --plt-min-ratio 0.2 --plt-max-ratio 1.0`

- `act_map_exp/scripts/visualize_quiver_progress.py` -> RViz animation of FoV
  optimization iterations from `quivers*.txt`.
  - Example (path-yaw top):  
    `python3 ../act_map_exp/scripts/visualize_quiver_progress.py --view top --variant none --path-yaw --rate 2.0`
  - Example (direct folder):  
    `python3 ../act_map_exp/scripts/visualize_quiver_progress.py --run-dir ../act_map_exp/trace_r2_a20/top/top_none/optimized_path_yaw --rate 2.0`
  - ROS2 note: run the script directly after `source /opt/ros/humble/setup.bash`
    and use `rviz2` with a MarkerArray display on `/stamped_pose_viz`.


## Config

- Tuning config: `optuna_fov_tune.yaml`
- Result outputs (paths depend on the trace root you use):
  - `act_map_exp/trace_r2_a20/optuna_trials.jsonl`
  - `act_map_exp/trace_r2_a20/optuna_best_params.yaml`
  - `act_map_exp/trace_r2_a20/optuna_pareto_params.yaml`

## Trace Roots

Use these paths when you want to pass explicit arguments.

r2_a20:
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/trace_r2_a20`
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r2_a20.yaml`

r1_a30:
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/trace_r1_a30`
`/home/shekoufeh/fov_ws/my_FIF-perception-aware-planning/act_map_exp/params/quad_rrt/warehouse/run_planner_defaults_r1_a30.yaml`

## Notes

- Registration pipeline lives in `act_map_exp/scripts/run_planner_exp.py`.
- `optimize.sh` reads best params from `best_params_yaml` in
  `optuna_fov_tune.yaml` (falls back to `initial_params`/`fixed_params` if the best file is missing).
- `optuna_fov_tune.yaml` points `workdir` to `act_map_exp/scripts`, so the
  relative `run_fov_opt_rrt_none.sh` command still works.
