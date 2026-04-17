#!/usr/bin/env python3

import argparse
import math
import os
import subprocess
import tempfile
import yaml


def _abs(p):
    return os.path.abspath(os.path.expanduser(p))


def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _save_yaml(path, data):
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False)


def _gen_offsets(max_xy, step_xy, max_z=0.0, step_z=0.0):
    offsets = []
    if step_xy <= 0:
        step_xy = max_xy
    if max_z <= 0 or step_z <= 0:
        z_vals = [0.0]
    else:
        z_vals = []
        z = -max_z
        while z <= max_z + 1e-9:
            z_vals.append(z)
            z += step_z
    x = -max_xy
    while x <= max_xy + 1e-9:
        y = -max_xy
        while y <= max_xy + 1e-9:
            for z in z_vals:
                offsets.append((x, y, z))
            y += step_xy
        x += step_xy
    offsets.sort(key=lambda v: v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return offsets


def _gen_yaw_offsets(max_yaw, step_yaw):
    if max_yaw <= 0 or step_yaw <= 0:
        return [0.0]
    vals = []
    yaw = -max_yaw
    while yaw <= max_yaw + 1e-9:
        vals.append(yaw)
        yaw += step_yaw
    vals.sort(key=lambda v: abs(v))
    return vals


def _infer_map_type(variation_yaml):
    node = variation_yaml.get("node", "")
    if "gp_trace" in node:
        return "gp_trace"
    if "gp_info" in node:
        return "gp_info"
    if "quadratic_trace" in node:
        return "quad_trace"
    if "quadratic_info" in node:
        return "quad_info"
    return ""


def _run_check(check_py, cfg_yaml, map_type, info_map_suffixes, extra_args):
    cmd = ["python2", check_py, "--config-yaml", cfg_yaml, "--only-invalid", "false"]
    if info_map_suffixes:
        cmd += ["--info-map-suffixes", info_map_suffixes]
    if map_type:
        cmd += ["--map-type", map_type]
    cmd += extra_args
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = proc.stdout
    valid = False
    saw_invalid = False
    for line in out.splitlines():
        if line.strip().startswith("=>"):
            if "INVALID" in line:
                saw_invalid = True
            elif "VALID" in line:
                valid = True
    if saw_invalid:
        return False, out
    return valid, out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-yaml", required=True, help="e.g. mid.yaml")
    parser.add_argument("--variation-yaml", action="append", required=True,
                        help="Repeat for each variation yaml")
    parser.add_argument("--info-map-suffixes", default="r2_a20")
    parser.add_argument("--map-type", default="", help="gp_trace/gp_info/quad_trace/quad_info")
    parser.add_argument("--adjust", choices=["start", "end", "both"], default="start")
    parser.add_argument("--max-offset", type=float, default=3.0)
    parser.add_argument("--step", type=float, default=0.5)
    parser.add_argument("--max-z-offset", type=float, default=0.0)
    parser.add_argument("--step-z", type=float, default=0.0)
    parser.add_argument("--max-yaw", type=float, default=30.0)
    parser.add_argument("--step-yaw", type=float, default=10.0)
    parser.add_argument("--max-tries", type=int, default=500)
    parser.add_argument("--extra-arg", action="append", default=[], help="extra args passed to check_rrt_validity.py")
    args = parser.parse_args()

    base_path = _abs(args.base_yaml)
    base = _load_yaml(base_path)
    var_paths = [_abs(v) for v in args.variation_yaml]
    vars_loaded = [_load_yaml(v) for v in var_paths]

    if not args.map_type and len(vars_loaded) == 1:
        args.map_type = _infer_map_type(vars_loaded[0])

    start0 = base["start"]
    end0 = base["end"]
    syaw0 = base.get("start_yaw_deg", 0.0)
    eyaw0 = base.get("end_yaw_deg", 0.0)

    offsets = _gen_offsets(args.max_offset, args.step, args.max_z_offset, args.step_z)
    yaw_offsets = _gen_yaw_offsets(args.max_yaw, args.step_yaw)

    check_py = os.path.join(os.path.dirname(__file__), "check_rrt_validity.py")
    if not os.path.exists(check_py):
        raise RuntimeError("check_rrt_validity.py not found: {}".format(check_py))

    tries = 0
    with tempfile.TemporaryDirectory(prefix="rrt_valid_search_") as td:
        base_tmp = os.path.join(td, "base.yaml")
        cfg_tmp = os.path.join(td, "cfg.yaml")
        _save_yaml(cfg_tmp, {
            "search_cfg": {
                "base": {base_tmp: "search"},
                "var": var_paths,
            }
        })

        for dstart in offsets:
            for dyaw_s in yaw_offsets:
                for dend in offsets if args.adjust in ("end", "both") else [(0.0, 0.0, 0.0)]:
                    for dyaw_e in yaw_offsets if args.adjust in ("end", "both") else [0.0]:
                        if args.adjust == "end":
                            dstart_use = (0.0, 0.0, 0.0)
                            dyaw_s_use = 0.0
                            dend_use = dend
                            dyaw_e_use = dyaw_e
                        elif args.adjust == "both":
                            dstart_use = dstart
                            dyaw_s_use = dyaw_s
                            dend_use = dend
                            dyaw_e_use = dyaw_e
                        else:
                            dstart_use = dstart
                            dyaw_s_use = dyaw_s
                            dend_use = (0.0, 0.0, 0.0)
                            dyaw_e_use = 0.0

                        base["start"] = [
                            start0[0] + dstart_use[0],
                            start0[1] + dstart_use[1],
                            start0[2] + dstart_use[2],
                        ]
                        base["end"] = [
                            end0[0] + dend_use[0],
                            end0[1] + dend_use[1],
                            end0[2] + dend_use[2],
                        ]
                        base["start_yaw_deg"] = syaw0 + dyaw_s_use
                        base["end_yaw_deg"] = eyaw0 + dyaw_e_use

                        _save_yaml(base_tmp, base)
                        tries += 1
                        ok, out = _run_check(check_py, cfg_tmp, args.map_type,
                                             args.info_map_suffixes, args.extra_arg)
                        # require all variations to be VALID: no INVALID lines
                        if ok:
                            print("VALID FOUND after {} tries".format(tries))
                            print("start:", base["start"], "start_yaw_deg:", base["start_yaw_deg"])
                            print("end:", base["end"], "end_yaw_deg:", base["end_yaw_deg"])
                            print(out)
                            return 0
                        if tries >= args.max_tries:
                            print("Reached max tries ({}) without VALID".format(args.max_tries))
                            return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
