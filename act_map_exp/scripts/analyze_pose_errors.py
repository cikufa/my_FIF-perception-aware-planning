#!/usr/bin/env python3

import os
import shutil
import argparse
from colorama import init, Fore
import yaml
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

from wandb_utils import add_wandb_args, safe_init, log_images, log_metrics, finish

init(autoreset=True)

pose_e_nm = 'pose_errors.txt'
Twc_nm = 'stamped_Twc.txt'

rc('font', **{'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)

def _replaceNan(values, rep):
    return [rep if math.isnan(v) else v for v in values]

def _loadPoses(pose_fn):
    times = []
    Twc = []
    data = np.loadtxt(pose_fn)
    assert data.shape[1] == 17

    times = data[:, 0].tolist()
    for l in data:
        Twc.append(l[1:].reshape((4, 4)))
    return times, Twc


def _loadPoseError(err_fn, max_trans_e_m=float('nan'),
                   max_rot_e_deg=float('nan')):
    names = []
    trans_e_m = []
    rot_e_deg = []
    with open(err_fn) as f:
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith('#'):
                continue
            elems = line.strip().split(' ')
            # assert len(elems) == 3
            names.append(elems[0])

            te_i = float(elems[1])
            if not math.isnan(max_trans_e_m) and te_i > max_trans_e_m:
                trans_e_m.append(float('nan'))
            else:
                trans_e_m.append(te_i)

            re_i = float(elems[2])
            if not math.isnan(max_rot_e_deg) and re_i > max_rot_e_deg:
                rot_e_deg.append(float('nan'))
            else:
                rot_e_deg.append(re_i)

    return names, trans_e_m, rot_e_deg


def _poseErrorHasData(err_fn):
    if not os.path.exists(err_fn):
        return False
    with open(err_fn) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            return True
    return False


def _summarize_errors(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {}
    nan_mask = np.isnan(arr)
    valid = arr[~nan_mask]
    if valid.size == 0:
        return {
            'count': 0,
            'nan_count': int(nan_mask.sum()),
        }
    return {
        'count': int(valid.size),
        'nan_count': int(nan_mask.sum()),
        'mean': float(np.mean(valid)),
        'median': float(np.median(valid)),
        'p95': float(np.percentile(valid, 95)),
        'max': float(np.max(valid)),
    }


def _inferTypesFromSubdirs(subdir_nms, base_cfg):
    if not base_cfg:
        return {}
    type_candidates = []
    if 'ordered_types' in base_cfg:
        type_candidates = list(base_cfg['ordered_types'])
    elif 'labels' in base_cfg:
        type_candidates = list(base_cfg['labels'].keys())
    if not type_candidates:
        return {}

    type_candidates = sorted(type_candidates, key=len, reverse=True)
    inferred = {}
    for nm in subdir_nms:
        if nm.endswith('_optimized_path_yaw') or nm == 'optimized_path_yaw':
            inferred[nm] = 'optimized_path_yaw'
            continue
        nm_check = nm[:-9] if nm.endswith('_path_yaw') else nm
        matched = None
        for t in type_candidates:
            if nm_check == t or nm_check.endswith('_' + t):
                matched = t
                break
        if matched:
            inferred[nm] = matched
    return inferred


def _ensureOptimizedPathYawStyle(cfg):
    if 'labels' not in cfg or 'colors' not in cfg or 'linestyles' not in cfg:
        return
    if 'optimized_path_yaw' not in cfg['labels']:
        cfg['labels']['optimized_path_yaw'] = 'optimized_path_yaw'
    if 'optimized_path_yaw' not in cfg['colors']:
        cfg['colors']['optimized_path_yaw'] = cfg['colors'].get('optimized', 'orange')
    if 'optimized_path_yaw' not in cfg['linestyles']:
        cfg['linestyles']['optimized_path_yaw'] = 'dashed'
    if 'ordered_types' in cfg and 'optimized_path_yaw' not in cfg['ordered_types']:
        if 'optimized' in cfg['ordered_types']:
            idx = cfg['ordered_types'].index('optimized') + 1
            cfg['ordered_types'].insert(idx, 'optimized_path_yaw')
        else:
            cfg['ordered_types'].append('optimized_path_yaw')


def _collectSubdirs(cfg_dir):
    subdir_map = {}
    for name in sorted(os.listdir(cfg_dir)):
        path = os.path.join(cfg_dir, name)
        if os.path.isdir(path):
            subdir_map[name] = path
    for name, path in list(subdir_map.items()):
        opt_dir = os.path.join(path, 'optimized')
        if os.path.isdir(opt_dir):
            opt_name = name + '_optimized'
            if opt_name not in subdir_map:
                subdir_map[opt_name] = opt_dir
        opt_path_yaw_dir = os.path.join(path, 'optimized_path_yaw')
        if os.path.isdir(opt_path_yaw_dir):
            opt_name = name + '_optimized_path_yaw'
            if opt_name not in subdir_map:
                subdir_map[opt_name] = opt_path_yaw_dir
    return subdir_map


def _hasPathYawResults(top_dir):
    for root, _, files in os.walk(top_dir):
        if 'pose_errors_path_yaw.txt' not in files:
            continue
        pose_err = os.path.join(root, 'pose_errors_path_yaw.txt')
        twc_path = os.path.join(root, 'stamped_Twc_path_yaw.txt')
        if os.path.exists(twc_path) and _poseErrorHasData(pose_err):
            return True
    return False


def _accumulateOptimizedPathYaw(top_dir, acc_trans_e, acc_rot_e, base_cfg):
    if not base_cfg:
        return
    max_trans = base_cfg.get('max_trans_e_m', base_cfg.get('hist_max_trans_e', float('nan')))
    max_rot = base_cfg.get('max_rot_e_deg', base_cfg.get('hist_max_rot_e', float('nan')))
    for root, _, files in os.walk(top_dir):
        if 'pose_errors_path_yaw.txt' not in files:
            continue
        if os.path.basename(root) != 'optimized_path_yaw':
            continue
        err_fn = os.path.join(root, 'pose_errors_path_yaw.txt')
        if not _poseErrorHasData(err_fn):
            continue
        _, trans_e_i, rot_e_i = _loadPoseError(err_fn, max_trans, max_rot)
        acc_trans_e.setdefault('optimized_path_yaw', []).extend(trans_e_i)
        acc_rot_e.setdefault('optimized_path_yaw', []).extend(rot_e_i)


def analyzeSingleCfg(cfg_dir, hide_x=False, base_cfg=None,
                     wandb_mod=None, wandb_prefix="",
                     pose_e_name=None, twc_name=None, plot_suffix=""):
    if pose_e_name is None:
        pose_e_name = pose_e_nm
    if twc_name is None:
        twc_name = Twc_nm
    print(Fore.RED + "==== process configuration {} ====".format(cfg_dir))
    analysis_cfg_fn = os.path.join(cfg_dir, 'analysis_cfg.yaml')
    subdir_map = _collectSubdirs(cfg_dir)
    subdir_nms = sorted(subdir_map.keys())
    if os.path.exists(analysis_cfg_fn):
        with open(analysis_cfg_fn) as f:
            ana_cfg = yaml.load(f, Loader=yaml.FullLoader)
        print("Found analysis configuration {}".format(ana_cfg))
        if base_cfg:
            ana_cfg.update(base_cfg)
            print("Effective analysis configuration {}".format(ana_cfg))
    else:
        ana_cfg = {}
        if base_cfg:
            ana_cfg.update(base_cfg)
        ana_cfg['types'] = _inferTypesFromSubdirs(subdir_nms, base_cfg)
        max_trans_default = float('nan')
        max_rot_default = float('nan')
        if base_cfg:
            max_trans_default = base_cfg.get(
                'max_trans_e_m', base_cfg.get('hist_max_trans_e', float('nan')))
            max_rot_default = base_cfg.get(
                'max_rot_e_deg', base_cfg.get('hist_max_rot_e', float('nan')))
        ana_cfg.setdefault('max_trans_e_m', max_trans_default)
        ana_cfg.setdefault('max_rot_e_deg', max_rot_default)
        if not ana_cfg['types']:
            print(Fore.YELLOW + "Missing analysis_cfg.yaml and could not infer types; skipping {}.".format(
                cfg_dir))
            return {}, {}
        print(Fore.YELLOW + "analysis_cfg.yaml not found; inferred types for {}.".format(cfg_dir))

    if 'types' in ana_cfg and ana_cfg['types']:
        missing_types = [v for v in subdir_nms if v not in ana_cfg['types']]
        if missing_types:
            inferred = _inferTypesFromSubdirs(
                missing_types, base_cfg if base_cfg else ana_cfg)
            ana_cfg['types'].update(inferred)
    if 'types' in ana_cfg and 'optimized_path_yaw' in ana_cfg['types'].values():
        _ensureOptimizedPathYawStyle(ana_cfg)

    if 'types' not in ana_cfg or not ana_cfg['types']:
        print(Fore.YELLOW + "No type mapping found for {}; skipping.".format(cfg_dir))
        return {}, {}

    missing_types = [v for v in subdir_nms if v not in ana_cfg['types']]
    if missing_types:
        print(Fore.YELLOW + "Skipping directories without type mapping: {}".format(
            ", ".join(missing_types)))
    subdir_nms = [v for v in subdir_nms if v in ana_cfg['types']]
    valid_entries = []
    for nm in subdir_nms:
        subdir = subdir_map[nm]
        pose_e_f_i = os.path.join(subdir, pose_e_name)
        if not _poseErrorHasData(pose_e_f_i):
            print(Fore.YELLOW + "Skip {} (missing or empty {}).".format(
                subdir, pose_e_name))
            continue
        valid_entries.append((nm, subdir, pose_e_f_i))
    if not valid_entries:
        print(Fore.YELLOW + "No pose errors found under {}; skipping.".format(cfg_dir))
        return {}, {}
    subdir_nms = [v[0] for v in valid_entries]
    subdirs = [v[1] for v in valid_entries]
    pose_e_fns = [v[2] for v in valid_entries]
    print("Going to analyze variations {}".format(subdir_nms))

    # load pose errors
    times = []
    trans_e_m = []
    rot_e_deg = []
    for idx, v in enumerate(subdirs):
        print("- Process {}...".format(v))
        pose_e_f_i = pose_e_fns[idx]
        _, trans_e_i, rot_e_i = _loadPoseError(
            pose_e_f_i, ana_cfg['max_trans_e_m'], ana_cfg['max_rot_e_deg'])
        times_i, _ = _loadPoses(os.path.join(v, twc_name))

        assert len(times_i) == len(trans_e_i), "{}: {} vs {}".format(
            v, len(times_i), len(trans_e_i))
        assert len(times_i) == len(rot_e_i), "{}: {} vs {}".format(
            v, len(times_i), len(rot_e_i))
        trans_e_m.append(trans_e_i)
        rot_e_deg.append(rot_e_i)
        times.append(times_i)

    fig = plt.figure(figsize=(16, 5))
    pos_ax = fig.add_subplot(121)
    if hide_x:
        pos_ax.get_xaxis().set_visible(False)
    pos_ax.set_ylabel('position error (m)')
    rot_ax = fig.add_subplot(122)
    rot_ax.set_ylabel('rotation error (deg)')
    if hide_x:
        rot_ax.get_xaxis().set_visible(False)
    for idx, nm in enumerate(subdir_nms):
        if not trans_e_m[idx]:
            print(Fore.RED + "Error for {} is empty, skip plotting.".format(nm))
            continue
        type_i = ana_cfg['types'][nm]
        pos_ax.plot(times[idx], trans_e_m[idx], label=ana_cfg['labels'][type_i],
                    color=ana_cfg['colors'][type_i], linestyle=ana_cfg['linestyles'][type_i])
        rot_ax.plot(times[idx], rot_e_deg[idx], label=ana_cfg['labels'][type_i],
                    color=ana_cfg['colors'][type_i], linestyle=ana_cfg['linestyles'][type_i])
    print('saving error plots...')
    plt.legend()
    fig.tight_layout()
    errors_plot_fn = os.path.join(cfg_dir, 'errors_comp{}.png'.format(plot_suffix))
    fig.savefig(errors_plot_fn, bbox_inches='tight', dpi=300)

    reg_rows = []
    for idx, nm in enumerate(subdir_nms):
        total = len(rot_e_deg[idx])
        if total == 0:
            reg_rows.append((nm, float('nan'), float('nan'), 0, 0))
            continue
        n_failed = sum([1 for v in rot_e_deg[idx] if math.isnan(v)])
        nonreg_ratio = 1.0 * n_failed / total
        reg_ratio = 1.0 - nonreg_ratio
        reg_rows.append((nm, nonreg_ratio, reg_ratio, n_failed, total))

    reg_fn = os.path.join(cfg_dir, 'registration_stats{}.csv'.format(plot_suffix))
    with open(reg_fn, 'w') as f:
        f.write('name,non_registered_ratio,registered_ratio,n_failed,n_total\n')
        for nm, nonreg_ratio, reg_ratio, n_failed, total in reg_rows:
            if math.isnan(nonreg_ratio):
                f.write("{},nan,nan,{},{}\n".format(nm, n_failed, total))
            else:
                f.write("{},{:.4f},{:.4f},{},{}\n".format(
                    nm, nonreg_ratio, reg_ratio, n_failed, total))

    if reg_rows:
        labels = [v[0] for v in reg_rows]
        ratios = [v[1] for v in reg_rows]
        fig_reg = plt.figure(figsize=(10, 4))
        ax = fig_reg.add_subplot(111)
        ax.bar(range(len(labels)), ratios, color='gray')
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=45, ha='right')
        ax.set_ylabel('non-registered ratio')
        fig_reg.tight_layout()
        reg_plot_fn = os.path.join(cfg_dir, 'registration_stats{}.png'.format(plot_suffix))
        fig_reg.savefig(reg_plot_fn, bbox_inches='tight', dpi=300)

    trans_e_raw = {}
    rot_e_raw = {}
    for idx, nm in enumerate(subdir_nms):
        if os.path.exists(pose_e_fns[idx]):
            _, trans_e_i_raw, rot_e_i_raw = _loadPoseError(pose_e_fns[idx])
        else:
            trans_e_i_raw = []
            rot_e_i_raw = []

        trans_e_raw[ana_cfg['types'][nm]] = trans_e_i_raw
        rot_e_raw[ana_cfg['types'][nm]] = rot_e_i_raw

    print(Fore.GREEN + '< Done.')

    if wandb_mod is not None:
        prefix = wandb_prefix.rstrip('/')
        if prefix:
            prefix = prefix + '/'
        metrics = {}
        for type_key, values in trans_e_raw.items():
            stats = _summarize_errors(values)
            for stat_key, stat_val in stats.items():
                metrics["{}trans_{}/{}".format(prefix, stat_key, type_key)] = stat_val
        for type_key, values in rot_e_raw.items():
            stats = _summarize_errors(values)
            for stat_key, stat_val in stats.items():
                metrics["{}rot_{}/{}".format(prefix, stat_key, type_key)] = stat_val
        log_metrics(wandb_mod, metrics)
        plot_key = 'errors_comp{}'.format(plot_suffix)
        log_images(wandb_mod, {prefix + plot_key: errors_plot_fn})

    return trans_e_raw, rot_e_raw


def _analyzeMultipleCfgs(top_dir, base_ana_cfg, args, wandb_mod=None,
                         pose_e_name=None, twc_name=None, plot_suffix=""):
    cfg_nms = [v for v in sorted(os.listdir(top_dir))
               if os.path.isdir(os.path.join(top_dir, v))]
    cfg_dirs = [os.path.join(top_dir, v) for v in cfg_nms]
    print(Fore.YELLOW + "1. Analyzing configurations under {}:".format(top_dir))
    for v in cfg_nms:
        print(Fore.YELLOW + "- {}".format(v))
    acc_trans_e = {}
    acc_rot_e = {}
    for cfg_d_i in cfg_dirs:
        cfg_key = os.path.basename(cfg_d_i.rstrip(os.sep))
        trans_e_i, rot_e_i = analyzeSingleCfg(
            cfg_d_i, base_cfg=base_ana_cfg, wandb_mod=wandb_mod, wandb_prefix=cfg_key,
            pose_e_name=pose_e_name, twc_name=twc_name, plot_suffix=plot_suffix)
        for k, v in trans_e_i.items():
            if k not in acc_trans_e:
                acc_trans_e[k] = v
            else:
                acc_trans_e[k].extend(v)
        for k, v in rot_e_i.items():
            if k not in acc_rot_e:
                acc_rot_e[k] = v
            else:
                acc_rot_e[k].extend(v)
    _accumulateOptimizedPathYaw(top_dir, acc_trans_e, acc_rot_e, base_ana_cfg)
    if 'optimized_path_yaw' in acc_trans_e or 'optimized_path_yaw' in acc_rot_e:
        _ensureOptimizedPathYawStyle(base_ana_cfg)
    print(Fore.YELLOW + "<<< Finished all configurations.")
    print(Fore.YELLOW + "2. Gathered translation and rotation errors:")
    print("- translation errors:")
    for k, v in acc_trans_e.items():
        print('  - {}: {}'.format(k, len(v)))
    print("- rotation errors:")
    for k, v in acc_rot_e.items():
        print('  - {}: {}'.format(k, len(v)))

    reg_overall = {}
    for k, v in acc_rot_e.items():
        total = len(v)
        if total == 0:
            reg_overall[k] = (float('nan'), float('nan'), 0, 0)
            continue
        n_failed = sum([1 for val in v if math.isnan(val)])
        nonreg_ratio = 1.0 * n_failed / total
        reg_ratio = 1.0 - nonreg_ratio
        reg_overall[k] = (nonreg_ratio, reg_ratio, n_failed, total)

    reg_fn = os.path.join(top_dir, 'registration_stats{}.csv'.format(plot_suffix))
    with open(reg_fn, 'w') as f:
        f.write('type,non_registered_ratio,registered_ratio,n_failed,n_total\n')
        for k in sorted(reg_overall.keys()):
            nonreg_ratio, reg_ratio, n_failed, total = reg_overall[k]
            if math.isnan(nonreg_ratio):
                f.write("{},nan,nan,{},{}\n".format(k, n_failed, total))
            else:
                f.write("{},{:.4f},{:.4f},{},{}\n".format(
                    k, nonreg_ratio, reg_ratio, n_failed, total))

    plot_max_trans_e = 1.2 * base_ana_cfg['hist_max_trans_e']
    plot_max_rot_e = 1.2 * base_ana_cfg['hist_max_rot_e']

    ordered_types = base_ana_cfg['ordered_types']
    trans_errors = [acc_trans_e.get(v, []) for v in ordered_types]
    trans_errors = [_replaceNan(v, plot_max_trans_e) for v in trans_errors]
    trans_labels = [base_ana_cfg['labels'][v] for v in ordered_types]
    trans_colors = [base_ana_cfg['colors'][v] for v in ordered_types]

    rot_errors = [acc_rot_e.get(v, []) for v in ordered_types]
    rot_errors = [_replaceNan(v, plot_max_rot_e) for v in rot_errors]
    rot_labels = [base_ana_cfg['labels'][v] for v in ordered_types]
    rot_colors = [base_ana_cfg['colors'][v] for v in ordered_types]

    fig = plt.figure(figsize=(12, 6))
    axes = fig.subplots(1, 2, sharey=True)
    pos_ax, rot_ax = axes[0], axes[1]
    for idx, v in enumerate(ordered_types):
        pos_ax.hist(trans_errors[idx], label=trans_labels[idx], bins=50,
                    color=trans_colors[idx], linestyle=base_ana_cfg['linestyles'][v],
                    density=True, histtype='step', cumulative=True,
                    range=(0, plot_max_trans_e))
        rot_ax.hist(rot_errors[idx], label=rot_labels[idx], bins=50,
                    color=rot_colors[idx], linestyle=base_ana_cfg['linestyles'][v],
                    density=True, histtype='step', cumulative=True,
                    range=(0, plot_max_rot_e))
    pos_ax.set_xlabel('Position error (m)')
    rot_ax.set_xlabel('Rotation error (deg)')

    pos_ax.set_xlim([0, base_ana_cfg['hist_max_trans_e']])
    pos_ax.set_ylim([args.plt_min_ratio, args.plt_max_ratio + 0.01])

    rot_ax.set_xlim([0, base_ana_cfg['hist_max_rot_e']])
    rot_ax.set_ylim([args.plt_min_ratio, args.plt_max_ratio + 0.01])

    ystep = (args.plt_max_ratio- args.plt_min_ratio) / 4.0
    y_ticks = np.arange(args.plt_min_ratio, args.plt_max_ratio + 0.001, ystep)
    y_ticklabels = ['{}%'.format(int(v * 100)) for v in y_ticks]

    pos_ax.set_yticks(y_ticks)
    pos_ax.set_yticklabels(y_ticklabels)

    if args.legend:
        plt.legend(loc='lower right', ncol=2, fontsize='small')

    fig.tight_layout()
    overall_hist_fn = os.path.join(top_dir, 'overall_hist{}.png'.format(plot_suffix))
    fig.savefig(overall_hist_fn, bbox_inches='tight', dpi=300)
    if wandb_mod is not None:
        log_images(wandb_mod, {'overall_hist{}'.format(plot_suffix): overall_hist_fn})

    ordered_types = base_ana_cfg['ordered_types']
    reg_labels = []
    reg_values = []
    for k in ordered_types:
        if k not in reg_overall:
            continue
        reg_labels.append(k)
        reg_values.append(reg_overall[k][0])
    if reg_labels:
        fig_reg = plt.figure(figsize=(10, 4))
        ax = fig_reg.add_subplot(111)
        ax.bar(range(len(reg_labels)), reg_values, color='gray')
        ax.set_xticks(range(len(reg_labels)))
        ax.set_xticklabels(reg_labels, rotation=45, ha='right')
        ax.set_ylabel('non-registered ratio')
        fig_reg.tight_layout()
        reg_plot_fn = os.path.join(top_dir, 'registration_stats{}.png'.format(plot_suffix))
        fig_reg.savefig(reg_plot_fn, bbox_inches='tight', dpi=300)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('top_dir', type=str,
                        help='top folder that contains different variations')
    parser.add_argument('--base_ana_cfg', type=str, required=True,
                        help='base analysis configuration')
    parser.add_argument('--multiple', action='store_true', dest='multiple',
                        help='how to treat the top_dir')
    parser.add_argument('--no_legend', action='store_false', dest='legend')
    parser.add_argument('--plt_min_ratio', type=float, default=0.0)
    parser.add_argument('--plt_max_ratio', type=float, default=1.0)
    add_wandb_args(parser)
    parser.set_defaults(multiple=False, legend=True)
    args = parser.parse_args()

    base_ana_cfg = None
    assert os.path.exists(args.base_ana_cfg)
    shutil.copy2(args.base_ana_cfg, os.path.join(args.top_dir, 'base_analysis_cfg.yaml'))
    with open(args.base_ana_cfg) as f:
        base_ana_cfg = yaml.load(f, Loader=yaml.FullLoader)
    print("Base configuration for analysis is {}".format(base_ana_cfg))

    run_name = os.path.basename(os.path.abspath(args.top_dir))
    run, wandb_mod = safe_init(
        args,
        config={'top_dir': args.top_dir, 'base_ana_cfg': args.base_ana_cfg,
                'multiple': args.multiple},
        run_name=run_name)

    if args.multiple:
        _analyzeMultipleCfgs(args.top_dir, base_ana_cfg, args, wandb_mod=wandb_mod)

    else:
        top_analysis_cfg_fn = os.path.join(args.top_dir, 'analysis_cfg.yaml')
        if not os.path.exists(top_analysis_cfg_fn):
            cfg_dirs = [v for v in sorted(os.listdir(args.top_dir))
                        if os.path.exists(os.path.join(args.top_dir, v, 'analysis_cfg.yaml'))]
            if cfg_dirs:
                print(Fore.YELLOW + "No analysis_cfg.yaml in top_dir; found per-config "
                      "analysis_cfg.yaml under subdirs. Enable --multiple behavior.")
                args.multiple = True
        if args.multiple:
            _analyzeMultipleCfgs(args.top_dir, base_ana_cfg, args, wandb_mod=wandb_mod)
        else:
            analyzeSingleCfg(args.top_dir, base_cfg=base_ana_cfg, wandb_mod=wandb_mod)

    finish(run)
