# FoV Optimization Helpers

This folder collects FoV optimization and tuning entrypoints without duplicating
pipeline logic. The heavy lifting stays in `act_map_exp/scripts`.

## Scripts

- `optimize.sh` -> calls `act_map_exp/scripts/run_fov_opt_rrt_none.sh`
  - Example: `./optimize.sh --both`

- `register.sh` -> calls `act_map_exp/scripts/run_planner_exp.py` with the
  warehouse configs by default.
  - Example (optimized only): `./register.sh --only_optimized`
  - Example (along path): `./register.sh --along_path --only_optimized`

- `tune.sh` -> calls `optuna_fov_tune.py` with `optuna_fov_tune.yaml`.
  - Example: `./tune.sh --mode tune`

## Config

- Tuning config: `optuna_fov_tune.yaml`
- Result outputs (unchanged, same as before):
  - `act_map_exp/trace/optuna_trials.jsonl`
  - `act_map_exp/trace/optuna_best_params.yaml`
  - `act_map_exp/trace/optuna_pareto_params.yaml`

## Notes

- Registration pipeline lives in `act_map_exp/scripts/run_planner_exp.py`.
- `optuna_fov_tune.yaml` points `workdir` to `act_map_exp/scripts`, so the
  relative `run_fov_opt_rrt_none.sh` command still works.
