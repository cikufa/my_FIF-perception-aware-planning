# FoV Optimization Helpers

This folder collects FoV optimization and tuning entrypoints without duplicating
pipeline logic. The heavy lifting stays in `act_map_exp/scripts`.

## Scripts

- `optimize.sh` -> calls `act_map_exp/scripts/run_fov_opt_rrt_none.sh`
  - Example: `./optimize.sh --both`

- `register.sh` -> registration wrapper for `run_planner_exp.py`.
  - Non-optimized variants: `./register.sh --variants`
  - Optimized only (normal): `./register.sh --optimized --normal`
  - Optimized only (along-path): `./register.sh --optimized --along-path`

- `tune.sh` -> tuning wrapper for `optuna_fov_tune.py`.
  - Tune normal: `./tune.sh --tune --normal`
  - Tune along-path: `./tune.sh --tune --along-path`
  - Fixed normal: `./tune.sh --fixed --normal`
  - Fixed along-path: `./tune.sh --fixed --along-path`

- `analyze_all.py` -> analyze all variations under `act_map_exp/trace`.
  - Example: `./analyze_all.py --plt-min-ratio 0.2 --plt-max-ratio 1.0`

- `analyze.sh` -> shell wrapper for `analyze_all.py`.
  - Example: `./analyze.sh --plt-min-ratio 0.2 --plt-max-ratio 1.0`


## Config

- Tuning config: `optuna_fov_tune.yaml`
- Result outputs (unchanged, same as before):
  - `act_map_exp/trace/optuna_trials.jsonl`
  - `act_map_exp/trace/optuna_best_params.yaml`
  - `act_map_exp/trace/optuna_pareto_params.yaml`

## Notes

- Registration pipeline lives in `act_map_exp/scripts/run_planner_exp.py`.
- In `--mode fixed`, if `fixed_params` is empty, the script auto-loads params
  from `best_params_yaml`.
- `optuna_fov_tune.yaml` points `workdir` to `act_map_exp/scripts`, so the
  relative `run_fov_opt_rrt_none.sh` command still works.
