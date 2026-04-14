import os


def add_wandb_args(parser):
    parser.add_argument(
        "--wandb",
        action="store_true",
        help="Enable Weights & Biases logging.",
    )
    parser.add_argument("--wandb_project", type=str, default=None)
    parser.add_argument("--wandb_entity", type=str, default=None)
    parser.add_argument("--wandb_group", type=str, default=None)
    parser.add_argument("--wandb_run_name", type=str, default=None)
    parser.add_argument("--wandb_tags", type=str, default=None,
                        help="Comma-separated tags.")
    parser.add_argument("--wandb_dir", type=str, default=None,
                        help="W&B run directory (for cache/artifacts).")
    parser.add_argument("--wandb_mode", type=str, default=None,
                        help="W&B mode: online, offline, or disabled.")


def _parse_tags(tags_str):
    if not tags_str:
        return None
    tags = [t.strip() for t in tags_str.split(",")]
    return [t for t in tags if t]


def safe_init(args, config=None, run_name=None):
    if not getattr(args, "wandb", False):
        return None, None
    try:
        import wandb
    except ImportError:
        print("wandb not installed; skipping W&B logging.")
        return None, None

    tags = _parse_tags(getattr(args, "wandb_tags", None))
    name = getattr(args, "wandb_run_name", None) or run_name
    init_kwargs = {
        "project": getattr(args, "wandb_project", None),
        "entity": getattr(args, "wandb_entity", None),
        "group": getattr(args, "wandb_group", None),
        "name": name,
        "tags": tags,
        "dir": getattr(args, "wandb_dir", None),
        "mode": getattr(args, "wandb_mode", None),
        "config": config,
    }
    init_kwargs = {k: v for k, v in init_kwargs.items() if v is not None}
    run = wandb.init(**init_kwargs)
    return run, wandb


def log_metrics(wandb_mod, metrics, step=None):
    if wandb_mod is None or not metrics:
        return
    if step is None:
        wandb_mod.log(metrics)
    else:
        wandb_mod.log(metrics, step=step)


def log_images(wandb_mod, images, step=None):
    if wandb_mod is None or not images:
        return
    payload = {}
    for key, path in images.items():
        if path and os.path.exists(path):
            payload[key] = wandb_mod.Image(path)
    if not payload:
        return
    if step is None:
        wandb_mod.log(payload)
    else:
        wandb_mod.log(payload, step=step)


def finish(run):
    if run is not None:
        run.finish()
