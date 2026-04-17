#!/usr/bin/env python2

import argparse
import os
import subprocess
import sys


def _repo_root():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(this_dir, "..", "..", ".."))

def _find_catkin_bin(repo_root):
    candidates = []
    # 1) explicit env
    devel = os.environ.get("CATKIN_DEVEL_PREFIX")
    if devel:
        candidates.append(os.path.join(devel, "lib", "act_map_exp", "check_rrt_validity_node"))
    # 2) workspace devel relative to repo_root
    candidates.append(os.path.join(repo_root, "devel", "lib", "act_map_exp", "check_rrt_validity_node"))
    # 3) walk up to find a devel/ folder
    cur = repo_root
    while True:
        cand = os.path.join(cur, "devel", "lib", "act_map_exp", "check_rrt_validity_node")
        candidates.append(cand)
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    # 4) common workspace location
    candidates.append("/home/shekoufeh/FIF_ws/devel/lib/act_map_exp/check_rrt_validity_node")
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


def _compile(src, out_bin, repo_root):
    includes = [
        repo_root,
        os.path.join(repo_root, "act_map", "include"),
        os.path.join(repo_root, "act_map_exp", "include"),
        os.path.join(repo_root, "third_party", "rpg_common", "include"),
        os.path.join(repo_root, "third_party", "rpg_vi_utils", "include"),
        os.path.join(repo_root, "unrealcv_bridge", "include"),
        os.path.join(repo_root, "build", "act_map"),
        os.path.join(repo_root, "src", "minkindr", "minkindr", "include"),
        "/usr/include/eigen3",
    ]
    # common workspace location for minkindr
    minkindr_ws = "/home/shekoufeh/FIF_ws/src/minkindr/minkindr/include"
    if os.path.isdir(minkindr_ws):
        includes.append(minkindr_ws)
    # generated protobuf headers in catkin build space
    build_act_map = "/home/shekoufeh/FIF_ws/build/act_map"
    if os.path.isdir(build_act_map):
        includes.append(build_act_map)
    cmd = [
        "g++",
        "-std=c++14",
        "-O2",
    ]
    for inc in includes:
        cmd.append("-I{}".format(inc))
    lib_dirs = [
        os.path.join(repo_root, "devel", "lib"),
        "/home/shekoufeh/FIF_ws/devel/lib",
        "/home/shekoufeh/FIF_ws/build/act_map",
        "/home/shekoufeh/FIF_ws/build/act_map_exp",
    ]
    for ld in lib_dirs:
        if os.path.isdir(ld):
            cmd.append("-L{}".format(ld))
    # add opencv flags if available to avoid linker errors
    def _pkg_config_flags(pkg):
        try:
            out = subprocess.check_output(["pkg-config", "--libs", pkg]).decode("utf-8")
            return out.strip().split()
        except Exception:
            return []

    opencv_flags = _pkg_config_flags("opencv4")
    if not opencv_flags:
        opencv_flags = _pkg_config_flags("opencv")

    cmd += [
        src,
        "-lact_map",
        "-lact_map_proto",
        "-lrpg_common",
        "-lglog",
        "-lprotobuf",
        "-lyaml-cpp",
    ]
    cmd += opencv_flags
    cmd += [
        "-pthread",
        "-o",
        out_bin,
    ]
    print(">> {}".format(" ".join(cmd)))
    return subprocess.call(cmd, cwd=repo_root)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rrt-dir", default=None)
    parser.add_argument("--config-yaml", default=None)
    parser.add_argument("--variation-dir", default=None)
    parser.add_argument("--esdf", default=None)
    parser.add_argument("--act-map-params", default=None)
    parser.add_argument("--info-map-dir", default=None)
    parser.add_argument("--info-map-root", default=None)
    parser.add_argument("--info-map-suffix", default=None)
    parser.add_argument("--info-map-suffixes", default=None)
    parser.add_argument("--map-type", default=None)
    parser.add_argument("--gp-vis-dir", default=None)
    parser.add_argument("--robot-radius", default=None)
    parser.add_argument("--verbose", default=None)
    parser.add_argument("--only-invalid", default=None)
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--fallback-build", action="store_true",
                        help="If catkin binary is missing, build with g++ as fallback.")
    args, unknown = parser.parse_known_args()

    repo_root = _repo_root()
    src = os.path.join(repo_root, "act_map_exp", "src", "check_rrt_validity.cpp")
    out_bin = "/tmp/check_rrt_validity"
    catkin_bin = _find_catkin_bin(repo_root)

    if catkin_bin:
        out_bin = catkin_bin
    else:
        if args.no_build:
            print("catkin binary missing. Expected under <workspace>/devel/lib/act_map_exp/check_rrt_validity_node")
            print("Run: catkin build act_map_exp")
            return 1
        if not args.fallback_build:
            print("catkin binary missing. Expected under <workspace>/devel/lib/act_map_exp/check_rrt_validity_node")
            print("Run: catkin build act_map_exp")
            print("Or re-run with --fallback-build to build locally.")
            return 1
        needs_build = (not os.path.exists(out_bin)) or (os.path.getmtime(src) > os.path.getmtime(out_bin))
        if needs_build:
            ret = _compile(src, out_bin, repo_root)
            if ret != 0:
                return ret

    # Defaults for config + map suffixes
    if args.config_yaml is None:
        default_cfg = os.path.join(repo_root, "act_map_exp", "params", "quad_rrt", "warehouse", "warehouse_all.yaml")
        if os.path.exists(default_cfg):
            args.config_yaml = default_cfg

    if args.info_map_suffixes is None and args.info_map_suffix is None:
        root = args.info_map_root or os.path.join(repo_root, "act_map_exp", "exp_data", "warehouse_FIF")
        suffixes = []
        for s in ["r2_a20", "r1_a30"]:
            if os.path.isdir(os.path.join(root, "gp_info_" + s)) or os.path.isdir(os.path.join(root, "quad_info_" + s)):
                suffixes.append(s)
        if suffixes:
            args.info_map_suffixes = ",".join(suffixes)

    cmd = [out_bin]
    for k in [
        ("--rrt-dir", args.rrt_dir),
        ("--config-yaml", args.config_yaml),
        ("--variation-dir", args.variation_dir),
        ("--esdf", args.esdf),
        ("--act-map-params", args.act_map_params),
        ("--info-map-dir", args.info_map_dir),
        ("--info-map-root", args.info_map_root),
        ("--info-map-suffix", args.info_map_suffix),
        ("--info-map-suffixes", args.info_map_suffixes),
        ("--map-type", args.map_type),
        ("--gp-vis-dir", args.gp_vis_dir),
        ("--robot-radius", args.robot_radius),
        ("--verbose", args.verbose),
        ("--only-invalid", args.only_invalid),
    ]:
        if k[1]:
            cmd += [k[0], str(k[1])]
    cmd += unknown
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
