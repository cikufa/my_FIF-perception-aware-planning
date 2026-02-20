#!/usr/bin/env python2

import os
import argparse
import shutil
import shlex
import subprocess
import sys
from colorama import init, Fore

import yaml

init(autoreset=True)

VERBOSE = False
PROGRESS = False
SHOW_SUBPROCESS = False


def log(msg):
    if VERBOSE:
        print(msg)


def progress_bar(current, total, suffix=""):
    if not PROGRESS:
        return
    width = 30
    if total <= 0:
        total = 1
    filled = int(width * float(current) / float(total))
    bar = "#" * filled + "-" * (width - filled)
    sys.stdout.write("\r[{}] {}/{} {}".format(bar, current, total, suffix))
    sys.stdout.flush()


def progress_done():
    if not PROGRESS:
        return
    sys.stdout.write("\n")
    sys.stdout.flush()


def run_subprocess(cmd_tokens):
    if SHOW_SUBPROCESS:
        return subprocess.call(cmd_tokens)
    with open(os.devnull, 'w') as devnull:
        return subprocess.call(cmd_tokens, stdout=devnull, stderr=devnull)


def _parseConfig(cfg_fn):
    assert os.path.exists(cfg_fn)
    assert cfg_fn.endswith('.yaml')
    cfg_dir = os.path.dirname(os.path.abspath(cfg_fn))

    with open(cfg_fn, 'r') as f:
        all_cfgs = yaml.load(f, Loader=yaml.FullLoader)
    log(all_cfgs)

    all_base_names = []
    all_base_fns = []
    all_var_fns = []

    # for gk, cfg in all_cfgs.iteritems():
    for gk, cfg in all_cfgs.items():
        log(Fore.YELLOW + "=====> Group {}".format(gk))
        bases = []
        variations = []
        base_names = []

        # for b_i, n_i in cfg['base'].iteritems():
        for b_i, n_i in cfg['base'].items():
            bases.append(os.path.join(cfg_dir, b_i))
            base_names.append(n_i)
        for v in sorted(cfg['var']):
            variations.append(os.path.join(cfg_dir, v))

        log(Fore.YELLOW + "From configuraiton file {}:".format(cfg_fn))
        log(Fore.YELLOW + "- found {} bases".format(len(bases)))
        for base_idx, v in enumerate(bases):
            log("  - {}: {}".format(v, base_names[base_idx]))

        log(Fore.YELLOW + "- found {} variations".format(len(variations)))
        for v in variations:
            log("  - {}".format(v))
        all_base_fns.append(bases)
        all_base_names.append(base_names)
        all_var_fns.append(variations)
    log("debugggggggg ll_base_fns {} all_base_names {} ll_var_fns {}".format(
        all_base_fns, all_base_names, all_var_fns))
    return all_base_fns, all_base_names, all_var_fns


def _dir_has_images(dir_path):
    if not os.path.isdir(dir_path):
        return False
    img_exts = set(['.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'])
    for name in os.listdir(dir_path):
        if name.startswith('.'):
            continue
        if os.path.splitext(name)[1].lower() in img_exts:
            return True
    return False


def _parse_optimized_dir_specs(specs):
    named = {}
    paths = []
    if not specs:
        return named, paths
    flat_specs = []
    for spec in specs:
        if ',' in spec:
            flat_specs.extend([v for v in spec.split(',') if v])
        else:
            flat_specs.append(spec)
    for spec in flat_specs:
        spec = spec.strip()
        if not spec:
            continue
        if '=' in spec:
            name, path = spec.split('=', 1)
            named[name.strip()] = path.strip()
        else:
            paths.append(spec)
    return named, paths


def _limit_registration_lists(rel_image_list, rel_cam_list, max_reg_images):
    if max_reg_images is None or max_reg_images <= 0:
        return None
    if not os.path.exists(rel_image_list) or not os.path.exists(rel_cam_list):
        return None

    with open(rel_image_list, 'r') as f:
        img_lines = [ln for ln in f.readlines() if ln.strip()]
    with open(rel_cam_list, 'r') as f:
        cam_lines = [ln for ln in f.readlines() if ln.strip()]

    n = min(len(img_lines), len(cam_lines))
    if n == 0:
        return (0, 0)
    if n <= max_reg_images:
        return (n, n)

    if max_reg_images == 1:
        idx = [0]
    else:
        idx = [(i * (n - 1)) // (max_reg_images - 1) for i in range(max_reg_images)]

    img_sel = [img_lines[i] for i in idx]
    cam_sel = [cam_lines[i] for i in idx]

    with open(rel_image_list, 'w') as f:
        f.writelines(img_sel)
    with open(rel_cam_list, 'w') as f:
        f.writelines(cam_sel)

    return (n, len(img_sel))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config_yaml', type=str, help='yaml')

    parser.add_argument('--defaults_yaml', type=str, default=None,
                        help='optional yaml with defaults for top_outdir, colmap_script_dir, base_model, optimized_dirs')

    parser.add_argument('--top_outdir', type=str, default=None,
                        help='where to put the output dir for the experiment.')

    parser.add_argument('--colmap_script_dir', default=None,
                        help='where to find the scripts to use COLMAP')

    parser.add_argument('--base_model', default=None,
                        help='colmap workspace against which to localize')

    # parser.add_argument('--unrealcv_ini', required=True,
    #                     help='unrealcv configuration (for camera intrinsics)')

    parser.add_argument('--no_clear_output', action='store_false', dest='clear_output')

    parser.add_argument('--skip_plan', action='store_true', dest='skip_plan')
    parser.add_argument('--skip_render', action='store_true', dest='skip_render')
    parser.add_argument('--skip_reg', action='store_true', dest='skip_reg')
    parser.add_argument('--skip_eval', action='store_true', dest='skip_eval')
    parser.add_argument('--only_optimized', action='store_true',
                        help='when set, run steps only for variation '
                             "type 'optimized'")
    parser.add_argument('--skip_optimized', action='store_true',
                        help='skip variations with type optimized (including optimized_path_yaw)')
    parser.add_argument('--optimized_dir', type=str, default=None,
                        help='path to the optimized folder to use when '
                             '--only_optimized is set')
    parser.add_argument('--optimized_dirs', nargs='+', default=None,
                        help='list of optimized folders (PATH or name=PATH) '
                             'for --only_optimized; order should match base list')

    parser.add_argument('--min_num_inliers', type=int, default=10)
    parser.add_argument('--max_reg_images', type=int, default=0,
                        help='limit number of registration images per run (0 = use all)')
    parser.add_argument('--max_trans_e_m', type=float, default=0.5)
    parser.add_argument('--max_rot_e_deg', type=float, default=10.0)
    parser.add_argument('--render_width', type=int, default=None,
                        help='override render image width (only for rendering step)')
    parser.add_argument('--render_height', type=int, default=None,
                        help='override render image height (only for rendering step)')
    parser.add_argument('--render_fov', type=float, default=None,
                        help='override render horizontal FOV in degrees')
    parser.add_argument('--render_sleep', type=float, default=None,
                        help='override render sleep seconds between poses')

    parser.add_argument('--exp_nm_rm_sufix', type=str, default='_base')
    parser.add_argument('--along_path', action='store_true',
                        help='use stamped_Twc_ue_path_yaw and path_yaw output names')
    parser.add_argument('--verbose', action='store_true',
                        help='enable verbose logging')
    parser.add_argument('--show_subprocess', action='store_true',
                        help='show stdout/stderr from subprocess commands')
    parser.add_argument('--fail_on_error', action='store_true',
                        help='exit non-zero if any stage fails for any config')
    parser.add_argument('--no_progress', action='store_true',
                        help='disable progress bar')
    parser.add_argument('--progress', action='store_true',
                        help='enable progress bar')

    # parser.set_defaults(clear_output=True, skip_plan=True, skip_render=False, skip_reg=False,
    #                     skip_eval=False)
    parser.set_defaults(clear_output=False, skip_plan=False, skip_render=False, skip_reg=False,
                        skip_eval=False)
    args = parser.parse_args()
    VERBOSE = bool(args.verbose)
    PROGRESS = bool(args.progress)
    SHOW_SUBPROCESS = bool(args.show_subprocess)
    if args.defaults_yaml:
        if not os.path.exists(args.defaults_yaml):
            parser.error("defaults_yaml does not exist: {}".format(args.defaults_yaml))
        with open(args.defaults_yaml, 'r') as f:
            defaults_cfg = yaml.load(f, Loader=yaml.FullLoader) or {}
        if args.top_outdir is None:
            args.top_outdir = defaults_cfg.get('top_outdir')
        if args.colmap_script_dir is None:
            args.colmap_script_dir = defaults_cfg.get('colmap_script_dir')
        if args.base_model is None:
            args.base_model = defaults_cfg.get('base_model')
        if args.optimized_dirs is None and args.optimized_dir is None:
            opt_from_defaults = defaults_cfg.get('optimized_dirs')
            if isinstance(opt_from_defaults, dict):
                args.optimized_dirs = ["{}={}".format(k, v) for k, v in opt_from_defaults.items()]
            elif isinstance(opt_from_defaults, list):
                args.optimized_dirs = opt_from_defaults
            elif isinstance(opt_from_defaults, str):
                args.optimized_dirs = [opt_from_defaults]
    if not args.top_outdir:
        parser.error("--top_outdir is required (or set it in --defaults_yaml)")
    needs_colmap = not (args.skip_render and args.skip_reg and args.skip_eval)
    needs_base_model = not (args.skip_reg and args.skip_eval)
    if needs_colmap and not args.colmap_script_dir:
        parser.error("--colmap_script_dir is required (or set it in --defaults_yaml)")
    if needs_base_model and not args.base_model:
        parser.error("--base_model is required (or set it in --defaults_yaml)")
    opt_named, opt_list = _parse_optimized_dir_specs(args.optimized_dirs)
    if args.optimized_dir:
        if opt_named or opt_list:
            parser.error("Use either --optimized_dir or --optimized_dirs")
        opt_list = [args.optimized_dir]

    opt_only = args.only_optimized and (opt_named or opt_list)
    if args.only_optimized and not opt_only:
        parser.error("--only_optimized requires --optimized_dir or --optimized_dirs to be set")
    if args.only_optimized and args.skip_optimized:
        parser.error("--only_optimized and --skip_optimized are mutually exclusive")

    # if args.skip_render or args.skip_plan or args.skip_reg:
    #     print(Fore.YELLOW + "Not clearing stuff due to skipping steps.") 
    #     args.clear_output = False

    if needs_colmap:
        assert os.path.exists(args.colmap_script_dir)
        ren_list_sc = os.path.join(args.colmap_script_dir, 'my_render_ue.py')
        assert os.path.exists(ren_list_sc)
        gen_list_sc = os.path.join(args.colmap_script_dir, 'generate_img_rel_path.py')
        assert os.path.exists(gen_list_sc)
        reg_sc = os.path.join(args.colmap_script_dir, 'register_images_to_model.py')
        assert os.path.exists(reg_sc)
        cal_e_sc = os.path.join(args.colmap_script_dir, 'calculate_pose_errors.py')
        assert os.path.exists(cal_e_sc)
    else:
        ren_list_sc = None
        gen_list_sc = None
        reg_sc = None
        cal_e_sc = None

    if needs_base_model:
        assert os.path.exists(args.base_model)
        base_img_dir = os.path.join(args.base_model, 'images')
        assert os.path.exists(base_img_dir)

    all_base_fns, all_base_names, all_var_fns = _parseConfig(args.config_yaml)

    def _resolve_opt_dir(exp_nm, base_idx, base_names):
        if opt_named:
            return opt_named.get(exp_nm)
        if not opt_list:
            return None
        if len(opt_list) == 1:
            return opt_list[0]
        if len(opt_list) == len(base_names):
            return opt_list[base_idx]
        return None

    for base_fns, base_names, var_fns in zip(all_base_fns, all_base_names, all_var_fns):
        log(Fore.YELLOW + "Processing group: base {} with var. {}".format(base_fns, var_fns))
        failed_cfgs = []
        base_contexts = []
        for base_idx, base_f_i in enumerate(base_fns):
            log(Fore.RED + "==============================")
            log(Fore.RED + "===== Running base {}... =====".format(base_f_i))
            log(Fore.RED + "==============================")

            assert os.path.exists(base_f_i)
            exp_nm = base_names[base_idx]
            # print("exp_nmmmmmmmmmmmmmmmmmm", exp_nm)
            base_outdir_i = os.path.join(args.top_outdir, exp_nm)
            log("base_outdir_iiiiiiiii {}".format(base_outdir_i))
            # if not os.path.exists(base_outdir_i):
            #     os.makedirs(base_outdir_i)

            log(Fore.YELLOW + "Experiment {} from {} with variations: ".format(exp_nm, base_f_i))
            for v in var_fns:
                log("- {}".format(v))
            log("Results will be saved in {}".format(base_outdir_i))

            with open(base_f_i, 'r') as f:
                base_params = yaml.load(f, Loader=yaml.FullLoader)
            ana_cfg_fn = os.path.join(base_outdir_i, 'analysis_cfg.yaml')
            if os.path.exists(ana_cfg_fn):
                log(Fore.YELLOW + "Found prev. analysis_cfg.yaml, wil update it")
                with open(ana_cfg_fn, 'r') as f:
                    prev_ana_cfg = yaml.load(f, Loader=yaml.FullLoader)
                    var_dir_to_types = prev_ana_cfg['types']
                    log("- loaded types {}".format(var_dir_to_types))
            else:
                var_dir_to_types = {}
            # Build variation list depending on mode
            var_entries = []

            def _append_opt_entries(opt_dir):
                abs_opt_dir = os.path.abspath(opt_dir)
                added = False
                if os.path.exists(abs_opt_dir):
                    var_entries.append({
                        'name': 'optimized',
                        'type': 'optimized',
                        'outdir': abs_opt_dir,
                        'cfg_fn': None,
                        'path_yaw': False
                    })
                    added = True
                else:
                    log(Fore.RED + "Optimized directory does not exist: {}".format(abs_opt_dir))
                    failed_cfgs.append(base_f_i + '-optimized-missing')

                if args.along_path:
                    abs_opt_path_yaw = abs_opt_dir.rstrip(os.sep) + "_path_yaw"
                    if os.path.exists(abs_opt_path_yaw):
                        var_entries.append({
                            'name': 'optimized_path_yaw',
                            'type': 'optimized_path_yaw',
                            'outdir': abs_opt_path_yaw,
                            'cfg_fn': None,
                            'path_yaw': True
                        })
                        added = True
                    else:
                        log(Fore.RED + "Optimized path-yaw directory does not exist: {}".format(
                            abs_opt_path_yaw))
                        failed_cfgs.append(base_f_i + '-optimized-path-yaw-missing')

                return added

            if opt_only:
                if args.skip_optimized:
                    log(Fore.YELLOW + "Skip optimized entries (skip_optimized set).")
                    continue
                opt_dir = _resolve_opt_dir(exp_nm, base_idx, base_names)
                if opt_dir is None:
                    log(Fore.RED + "No optimized directory specified for base {}.".format(exp_nm))
                    failed_cfgs.append(base_f_i + '-optimized-missing')
                    continue
                if not _append_opt_entries(opt_dir):
                    continue
            else:
                for var_f_i in var_fns:
                    var_entries.append({
                        'cfg_fn': var_f_i,
                        'outdir': None,  # to be filled
                        'path_yaw': False
                    })
                if (not args.skip_optimized) and args.along_path and (opt_named or opt_list):
                    opt_dir = _resolve_opt_dir(exp_nm, base_idx, base_names)
                    if opt_dir is not None:
                        _append_opt_entries(opt_dir)

            base_contexts.append({
                'base_idx': base_idx,
                'base_f_i': base_f_i,
                'exp_nm': exp_nm,
                'base_outdir_i': base_outdir_i,
                'base_params': base_params,
                'var_entries': var_entries,
                'var_by_cfg': {e['cfg_fn']: e for e in var_entries if e.get('cfg_fn')},
                'var_dir_to_types': var_dir_to_types,
                'base_total': len(var_entries),
                'progress': 0,
            })

        def _run_var_entry(base_ctx, var_entry):
            base_ctx['progress'] += 1
            var_idx = base_ctx['progress']
            base_total = base_ctx['base_total']
            base_f_i = base_ctx['base_f_i']
            exp_nm = base_ctx['exp_nm']
            base_outdir_i = base_ctx['base_outdir_i']
            base_params = base_ctx['base_params']
            var_dir_to_types = base_ctx['var_dir_to_types']

            if var_entry.get('cfg_fn') is None:
                log(Fore.RED + "===== Running optimized folder {}... =====".format(var_entry['outdir']))
                if var_entry['name'] == 'optimized':
                    var_nm_i = exp_nm + '_optimized'
                else:
                    var_nm_i = exp_nm + '_' + var_entry['name']
                input_dir_i = var_entry['outdir']
                outdir_i = input_dir_i
                var_type = var_entry.get('type', 'optimized')
                entry_path_yaw = var_entry.get('path_yaw', False)
                fail_label = base_f_i + '-optimized'
            else:
                var_f_i = var_entry['cfg_fn']
                log(Fore.RED + "===== Running variation {}... =====".format(var_f_i))
                with open(var_f_i) as fvar:
                    var_opts_i = yaml.load(fvar, Loader=yaml.FullLoader)
                if args.only_optimized and var_opts_i.get('type') != 'optimized':
                    log(Fore.BLUE + "> Skip variation (only_optimized set).")
                    return
                if args.skip_optimized and var_opts_i.get('type') in ('optimized', 'optimized_path_yaw'):
                    log(Fore.BLUE + "> Skip variation (skip_optimized set).")
                    return
                var_nm_i = exp_nm + '_' + var_opts_i['type']
                outdir_i = os.path.abspath(os.path.join(base_outdir_i, var_nm_i))
                input_dir_i = outdir_i
                var_type = var_opts_i['type']
                entry_path_yaw = var_entry.get('path_yaw', False)
                if args.clear_output and os.path.exists(outdir_i):
                    shutil.rmtree(outdir_i)
                    log(Fore.RED + "Removed {}".format(outdir_i))
                fail_label = base_f_i + '-' + var_f_i
            log("- output: {}".format(outdir_i))
            if input_dir_i != outdir_i:
                log("- input:  {}".format(input_dir_i))
            var_dir_to_types[var_nm_i] = var_type
            progress_bar(var_idx, base_total, "{}:{} init".format(exp_nm, var_nm_i))
#________________________________________________________________________________________________________________

            print(Fore.YELLOW + "Step 1: plan and save")
            if args.skip_plan:
                print(Fore.BLUE + "> Skip planner")
            elif var_entry.get('cfg_fn') is None:
                print(Fore.BLUE + "> Skip planner (optimized entry)")
            else:
                cfg_i = os.path.join(outdir_i, var_nm_i + ".yaml")
                params_i = base_params.copy()
                params_i.update(var_opts_i)
                params_i['save_traj_abs_dir'] = outdir_i
                params_i['save_abs_dir'] = outdir_i
                if not os.path.exists(outdir_i):
                    os.makedirs(outdir_i)
                with open(cfg_i, 'w') as f:
                    yaml.dump(params_i, f, default_flow_style=False)
                print("- cfg: {}".format(cfg_i))
                set_planner_cmd = ("""rosservice call /{}/set_planner_state"""
                                   """ "config: '{}'" """.format(var_opts_i['node'], cfg_i))
                print(Fore.BLUE + set_planner_cmd)
                subprocess.call(shlex.split(set_planner_cmd))
                call_planner_cmd = "rosservice call /{}/plan_vis_save".format(var_opts_i['node'])
                print(Fore.BLUE + call_planner_cmd)
                subprocess.call(shlex.split(call_planner_cmd))

                reset_planner_cmd = "rosservice call /{}/reset_planner".format(var_opts_i['node'])
                print(Fore.BLUE + reset_planner_cmd)
                subprocess.call(shlex.split(reset_planner_cmd))

           
            
#________________________________________________________________________________________________________________

            path_suffix = '_path_yaw' if entry_path_yaw else ''
            progress_bar(var_idx, base_total, "{}:{} render".format(exp_nm, var_nm_i))
            render_dir_i = os.path.join(outdir_i, 'rendering')
            if not args.skip_render:
                ue_pose_candidates = [
                    os.path.join(input_dir_i, 'stamped_Twc_ue{}.txt'.format(path_suffix)),
                    os.path.join(input_dir_i, 'optimized_stamped_Twc_ue{}.txt'.format(path_suffix)),
                ]
                ue_pose_fn = None
                for candidate in ue_pose_candidates:
                    if os.path.exists(candidate):
                        ue_pose_fn = candidate
                        break

                if ue_pose_fn is None:
                    failed_cfgs.append(fail_label)
                    log(Fore.RED + "Cannot find UE poses (checked: {}). CONTINUE TO NEXT.".format(
                        ", ".join(ue_pose_candidates)))
                    return
                if not os.path.exists(outdir_i):
                    os.makedirs(outdir_i)

                render_cmd =  "{} {} --save_dir {}".format(
                    ren_list_sc, ue_pose_fn, render_dir_i)
                if args.render_width is not None:
                    render_cmd += " --width {}".format(args.render_width)
                if args.render_height is not None:
                    render_cmd += " --height {}".format(args.render_height)
                if args.render_fov is not None:
                    render_cmd += " --fov {}".format(args.render_fov)
                if args.render_sleep is not None:
                    render_cmd += " --sleep {}".format(args.render_sleep)
             
                # render_cmd = ("rosrun unrealcv_bridge render_from_poses.py {} --save_dir {}"
                #               " --save_sleep_sec 0.1 --unrealcv_in {}").format(
                #                   ue_pose_fn, render_dir_i, args.unrealcv_ini)

                log(Fore.BLUE + render_cmd)
                render_ret = run_subprocess(shlex.split(render_cmd))
                if render_ret != 0:
                    failed_cfgs.append(fail_label + '-render-failed')
                    log(Fore.RED + "Rendering failed for {} (exit code {}).".format(
                        outdir_i, render_ret))
                    return
            else:
                log(Fore.BLUE + "> Skip rendering.")

            reg_name = var_nm_i
            if path_suffix and not reg_name.endswith(path_suffix):
                reg_name = reg_name + path_suffix
            progress_bar(var_idx, base_total, "{}:{} register".format(exp_nm, var_nm_i))
            reg_img_dir_i = os.path.join(render_dir_i, 'images')
            reg_img_nm_to_cam = os.path.join(render_dir_i, 'img_nm_to_colmap_cam.txt')
            reg_missing = []
            if not os.path.isdir(render_dir_i):
                reg_missing.append(render_dir_i)
            if not os.path.isdir(reg_img_dir_i):
                reg_missing.append(reg_img_dir_i)
            elif not _dir_has_images(reg_img_dir_i):
                reg_missing.append(reg_img_dir_i + " (no images)")
            if not os.path.exists(reg_img_nm_to_cam):
                reg_missing.append(reg_img_nm_to_cam)
            if not args.skip_reg:
                if reg_missing:
                    log(Fore.YELLOW + "Skip registration for {} (missing: {}).".format(
                        outdir_i, ", ".join(reg_missing)))
                    failed_cfgs.append(fail_label + '-reg-missing')
                    log(Fore.BLUE + "> Skip image registration.")
                    reg_success = False
                else:
                    gen_list_cmd = "{} --base_dir {} --img_dir {} --img_nm_to_cam_list {}".format(
                        gen_list_sc, base_img_dir, reg_img_dir_i,
                        reg_img_nm_to_cam)
                    log(Fore.BLUE + gen_list_cmd)
                    gen_ret = run_subprocess(shlex.split(gen_list_cmd))
                    if gen_ret != 0:
                        log(Fore.RED + "generate_img_rel_path failed for {} (exit code {}).".format(
                            outdir_i, gen_ret))
                        failed_cfgs.append(fail_label + '-gen-list-failed')
                        return
                    rel_image_list = os.path.join(render_dir_i, 'images/rel_img_path.txt')
                    rel_cam_list = os.path.join(render_dir_i, 'images/rel_img_nm_to_cam_list.txt')
                    if not os.path.exists(rel_image_list) or not os.path.exists(rel_cam_list):
                        log(Fore.YELLOW + "Skip registration for {} (missing: {}{}).".format(
                            outdir_i,
                            rel_image_list if not os.path.exists(rel_image_list) else "",
                            ", " + rel_cam_list if not os.path.exists(rel_cam_list) else ""))
                        failed_cfgs.append(fail_label + '-reg-missing')
                        log(Fore.BLUE + "> Skip image registration.")
                        reg_success = False
                    else:
                        if args.max_reg_images > 0:
                            limit_info = _limit_registration_lists(
                                rel_image_list, rel_cam_list, args.max_reg_images)
                            if limit_info is not None:
                                n_before, n_after = limit_info
                                if n_before > n_after:
                                    log(Fore.BLUE + "Limit registration images: {} -> {}".format(
                                        n_before, n_after))
                        reg_cmd = ("{} {} --reg_name {} --reg_list_fn {} "
                                   "--img_nm_to_colmap_cam_list {} --upref_no_time "
                                   "--min_num_inliers {}").format(
                                       reg_sc, args.base_model, reg_name, rel_image_list, rel_cam_list,
                                       args.min_num_inliers)
                        log(Fore.BLUE + reg_cmd)
                        reg_ret = run_subprocess(shlex.split(reg_cmd))
                        if reg_ret != 0:
                            log(Fore.RED + "Registration failed for {} (exit code {}).".format(
                                outdir_i, reg_ret))
                            failed_cfgs.append(fail_label + '-reg-failed')
                            reg_success = False
                            return
                        reg_success = True
            else:
                log(Fore.BLUE + "> Skip image registration.")
                reg_success = False

            progress_bar(var_idx, base_total, "{}:{} eval".format(exp_nm, var_nm_i))
            if not args.skip_eval:
                reg_model_dir_i = os.path.join(args.base_model, reg_name+"_sparse")
                eval_missing = []
                if not os.path.exists(reg_model_dir_i):
                    eval_missing.append(reg_model_dir_i)
                reg_img_name_to_colmap = os.path.join(render_dir_i, 'img_name_to_colmap_Tcw.txt')
                if not os.path.exists(reg_img_name_to_colmap):
                    eval_missing.append(reg_img_name_to_colmap)
                if not os.path.isdir(reg_img_dir_i):
                    eval_missing.append(reg_img_dir_i)
                elif not _dir_has_images(reg_img_dir_i):
                    eval_missing.append(reg_img_dir_i + " (no images)")
                if eval_missing:
                    log(Fore.YELLOW + "Skip evaluation for {} (missing: {}).".format(
                        outdir_i, ", ".join(eval_missing)))
                    failed_cfgs.append(fail_label + '-eval-missing')
                    return
                eval_outdir_i = outdir_i
                if entry_path_yaw:
                    eval_outdir_i = os.path.join(outdir_i, 'eval{}'.format(path_suffix))
                    if not os.path.exists(eval_outdir_i):
                        os.makedirs(eval_outdir_i)
                eval_cmd = ("{} --reg_model_dir {} --reg_img_name_to_colmap_Tcw {}"
                            " --reg_img_dir {} --output_path {}"
                            " --acc_trans_thresh {} --acc_rot_thresh {}").format(
                                cal_e_sc, reg_model_dir_i,
                                reg_img_name_to_colmap,
                                reg_img_dir_i, eval_outdir_i,
                                args.max_trans_e_m, args.max_rot_e_deg)
                log(Fore.BLUE + eval_cmd)
                eval_ret = run_subprocess(shlex.split(eval_cmd))
                if eval_ret != 0:
                    log(Fore.RED + "Pose error evaluation failed for {} (exit code {}).".format(
                        outdir_i, eval_ret))
                    failed_cfgs.append(fail_label + '-eval-failed')
                    return
                if entry_path_yaw:
                    for fn in ['pose_errors.txt', 'registered_poses_ue.txt']:
                        src_fn = os.path.join(eval_outdir_i, fn)
                        if not os.path.exists(src_fn):
                            continue
                        base_nm, ext = os.path.splitext(fn)
                        dst_fn = os.path.join(outdir_i, base_nm + path_suffix + ext)
                        if os.path.exists(dst_fn):
                            os.remove(dst_fn)
                        shutil.move(src_fn, dst_fn)
                    if os.path.exists(eval_outdir_i) and not os.listdir(eval_outdir_i):
                        os.rmdir(eval_outdir_i)
            else:
                log(Fore.BLUE + "> Skip evaluation.")
            progress_bar(var_idx, base_total, "{}:{} done".format(exp_nm, var_nm_i))
            if var_idx == base_total:
                progress_done()

        if opt_only:
            for base_ctx in base_contexts:
                for var_entry in base_ctx['var_entries']:
                    _run_var_entry(base_ctx, var_entry)
        else:
            for var_f_i in var_fns:
                for base_ctx in base_contexts:
                    var_entry = base_ctx['var_by_cfg'].get(var_f_i)
                    if var_entry is None:
                        continue
                    _run_var_entry(base_ctx, var_entry)
            for base_ctx in base_contexts:
                for var_entry in base_ctx['var_entries']:
                    if var_entry.get('cfg_fn') is None:
                        _run_var_entry(base_ctx, var_entry)

        for base_ctx in base_contexts:
            with open(os.path.join(base_ctx['base_outdir_i'], 'analysis_cfg.yaml'), 'w') as f:
                yaml.dump({'types': base_ctx['var_dir_to_types'],
                           'max_trans_e_m': args.max_trans_e_m,
                           'max_rot_e_deg': args.max_rot_e_deg}, f, default_flow_style=False)

    log(Fore.RED + "Failed configurations:")
    for v in failed_cfgs:
        log(Fore.RED + v)
    if failed_cfgs and args.fail_on_error:
        sys.exit(1)
