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
import json

from wandb_utils import add_wandb_args, safe_init, log_images, log_metrics, finish

init(autoreset=True)

pose_e_nm = 'pose_errors.txt'
Twc_nm = 'stamped_Twc.txt'
Twc_path_yaw_nm = 'stamped_Twc_path_yaw.txt'
pose_e_path_yaw_nm = 'pose_errors_path_yaw.txt'

rc('font', **{'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)

def _replaceNan(values, rep):
    return [rep if math.isnan(v) else v for v in values]

def _mode_values(values, mode, plot_max, penalty):
    if mode == 'finite':
        return [v for v in values if math.isfinite(v)]
    if mode == 'original':
        return _replaceNan(values, plot_max)
    return _replaceNan(values, penalty)

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

def _select_pose_error_file(subdir, default_err_name, default_twc_name):
    # Prefer path-yaw files when present in a folder.
    path_err = os.path.join(subdir, pose_e_path_yaw_nm)
    if os.path.exists(path_err):
        twc_path = os.path.join(subdir, Twc_path_yaw_nm)
        if not os.path.exists(twc_path):
            twc_path = os.path.join(subdir, default_twc_name)
        return path_err, twc_path
    err_path = os.path.join(subdir, default_err_name)
    twc_path = os.path.join(subdir, default_twc_name)
    if not os.path.exists(twc_path):
        twc_path_yaw = os.path.join(subdir, Twc_path_yaw_nm)
        if os.path.exists(twc_path_yaw):
            twc_path = twc_path_yaw
    return err_path, twc_path


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
        'rmse': float(np.sqrt(np.mean(valid ** 2))),
    }

def _penalty_value(values, fallback):
    if fallback is not None and math.isfinite(fallback):
        return float(fallback)
    arr = np.asarray(values, dtype=float)
    valid = arr[np.isfinite(arr)]
    if valid.size == 0:
        return 0.0
    return float(np.max(valid))

def _finite_stats(values):
    arr = np.asarray(values, dtype=float)
    valid = arr[np.isfinite(arr)]
    if valid.size == 0:
        return {
            'count': 0,
            'mean': float('nan'),
            'std': float('nan'),
            'max': float('nan'),
        }
    return {
        'count': int(valid.size),
        'mean': float(np.mean(valid)),
        'std': float(np.std(valid)),
        'max': float(np.max(valid)),
    }

def _ratio_under_threshold(values, threshold, include_nan=True):
    arr = np.asarray(values, dtype=float)
    total = int(arr.size)
    if total == 0:
        return float('nan'), 0, 0
    valid = np.isfinite(arr)
    under = np.logical_and(valid, arr <= threshold)
    under_count = int(np.sum(under))
    if include_nan:
        denom = total
    else:
        denom = int(np.sum(valid))
    ratio = float('nan') if denom == 0 else float(under_count / denom)
    return ratio, under_count, denom

def _summarize_errors_penalized(values, penalty):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {}
    nan_mask = np.isnan(arr)
    nan_count = int(nan_mask.sum())
    filled = np.array(arr, copy=True)
    filled[nan_mask] = penalty
    valid = filled[np.isfinite(filled)]
    if valid.size == 0:
        return {
            'count': int((~nan_mask).sum()),
            'nan_count': nan_count,
        }
    return {
        'count': int((~nan_mask).sum()),
        'nan_count': nan_count,
        'mean': float(np.mean(valid)),
        'std': float(np.std(valid)),
        'median': float(np.median(valid)),
        'p95': float(np.percentile(valid, 95)),
        'max': float(np.max(valid)),
        'rmse': float(np.sqrt(np.mean(valid ** 2))),
        'penalty': float(penalty),
    }

def _get_nested(d, *keys):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def _read_pose_metrics(metrics_fn):
    if not os.path.exists(metrics_fn):
        return None
    try:
        with open(metrics_fn, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(Fore.YELLOW + f"Failed to read metrics {metrics_fn}: {e}")
        return None


def _annotate_bar_values(ax, bars, fmt="{:.2f}", as_percent=False, fontsize=12):
    for rect in bars:
        height = rect.get_height()
        if not math.isfinite(height):
            continue
        label = "{:.1f}%".format(height * 100.0) if as_percent else fmt.format(height)
        ax.annotate(
            label,
            xy=(rect.get_x() + rect.get_width() / 2.0, height),
            xytext=(0, 3),
            textcoords="offset points",
            ha='center',
            va='bottom',
            fontsize=fontsize,
            clip_on=False,
        )


def _stats_with_defaults(values):
    stats = _summarize_errors(values)
    defaults = {
        'count': 0,
        'nan_count': 0,
        'mean': float('nan'),
        'median': float('nan'),
        'p95': float('nan'),
        'max': float('nan'),
        'rmse': float('nan'),
    }
    if not stats:
        return defaults
    for k, v in defaults.items():
        stats.setdefault(k, v)
    return stats

def _stats_with_defaults_penalized(values, penalty):
    penalty_val = _penalty_value(values, penalty)
    stats = _summarize_errors_penalized(values, penalty_val)
    defaults = {
        'count': 0,
        'nan_count': 0,
        'mean': float('nan'),
        'std': float('nan'),
        'median': float('nan'),
        'p95': float('nan'),
        'max': float('nan'),
        'rmse': float('nan'),
        'penalty': penalty_val,
    }
    if not stats:
        return defaults
    for k, v in defaults.items():
        stats.setdefault(k, v)
    return stats


def _fmt_stat(val):
    return 'nan' if not math.isfinite(val) else '{:.6g}'.format(val)

def _save_png(fig, out_fn, pad=0.25, dpi=300):
    fig.savefig(out_fn, bbox_inches='tight', pad_inches=pad, dpi=dpi)

def _finalize_reg_bar_plot(fig, ax, labels):
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha='right', rotation_mode='anchor')
    fig.tight_layout()
    fig.subplots_adjust(left=0.28, bottom=0.40)

def _finalize_two_row_plot(fig, bottom_ax, labels):
    bottom_ax.set_xticks(range(len(labels)))
    bottom_ax.set_xticklabels(labels, rotation=45, ha='right', rotation_mode='anchor')
    fig.tight_layout()
    fig.subplots_adjust(left=0.28, bottom=0.40, hspace=0.35)

def _finalize_table_plot(fig):
    fig.tight_layout()
    fig.subplots_adjust(left=0.28, right=0.98, top=0.98, bottom=0.02)

def _finalize_long_x_plot(fig, bottom_ax, labels):
    bottom_ax.set_xticks(range(len(labels)))
    bottom_ax.set_xticklabels(labels, rotation=90, ha='center')
    fig.tight_layout()
    fig.subplots_adjust(left=0.30, bottom=0.40, hspace=0.30)

def _ensure_mode_dirs(top_dir):
    base = os.path.join(top_dir, 'analysis_outputs')
    modes = {
        'finite': os.path.join(base, 'finite'),
        'original': os.path.join(base, 'original'),
        'penalized': os.path.join(base, 'penalized'),
    }
    for d in modes.values():
        os.makedirs(d, exist_ok=True)
    return modes


def _write_error_stats_csv(out_dir, plot_suffix, ordered_types, trans_stats, rot_stats):
    out_fn = os.path.join(out_dir, 'overall_error_stats{}.csv'.format(plot_suffix))
    with open(out_fn, 'w') as f:
        f.write('type,metric,count,nan_count,mean,median,p95,max\n')
        for k in ordered_types:
            for metric, stats in (('trans', trans_stats[k]), ('rot', rot_stats[k])):
                f.write('{},{},{},{},{},{},{},{}\n'.format(
                    k,
                    metric,
                    stats['count'],
                    stats['nan_count'],
                    _fmt_stat(stats['mean']),
                    _fmt_stat(stats['median']),
                    _fmt_stat(stats['p95']),
                    _fmt_stat(stats['max']),
                ))
    print(Fore.YELLOW + "Saved error stats to {}".format(out_fn))

def _write_error_stats_penalized_csv(out_dir, plot_suffix, ordered_types, trans_stats, rot_stats):
    out_fn = os.path.join(out_dir, 'overall_error_stats_penalized{}.csv'.format(plot_suffix))
    with open(out_fn, 'w') as f:
        f.write('type,metric,count,nan_count,penalty,mean,median,p95,max\n')
        for k in ordered_types:
            for metric, stats in (('trans', trans_stats[k]), ('rot', rot_stats[k])):
                f.write('{},{},{},{},{},{},{},{},{}\n'.format(
                    k,
                    metric,
                    stats['count'],
                    stats['nan_count'],
                    _fmt_stat(stats['penalty']),
                    _fmt_stat(stats['mean']),
                    _fmt_stat(stats['median']),
                    _fmt_stat(stats['p95']),
                    _fmt_stat(stats['max']),
                ))
    print(Fore.YELLOW + "Saved penalized error stats to {}".format(out_fn))


def _write_compare_vs_optimized(out_dir, plot_suffix, ordered_types, trans_stats, rot_stats):
    ref_type = 'optimized_path_yaw'
    if ref_type not in trans_stats or ref_type not in rot_stats:
        return
    out_fn = os.path.join(out_dir, 'overall_error_compare_vs_optimized{}.csv'.format(plot_suffix))
    with open(out_fn, 'w') as f:
        f.write('type,metric,stat,value,optimized_value,diff,ratio\n')
        for k in ordered_types:
            for metric, stats, ref_stats in (
                ('trans', trans_stats[k], trans_stats[ref_type]),
                ('rot', rot_stats[k], rot_stats[ref_type]),
            ):
                for stat in ('mean', 'median', 'p95', 'max'):
                    val = stats[stat]
                    ref_val = ref_stats[stat]
                    if math.isfinite(val) and math.isfinite(ref_val):
                        diff = val - ref_val
                        ratio = val / ref_val if ref_val != 0.0 else float('nan')
                    else:
                        diff = float('nan')
                        ratio = float('nan')
                    f.write('{},{},{},{},{},{},{}\n'.format(
                        k,
                        metric,
                        stat,
                        _fmt_stat(val),
                        _fmt_stat(ref_val),
                        _fmt_stat(diff),
                        _fmt_stat(ratio),
                    ))
    print(Fore.YELLOW + "Saved optimized comparison to {}".format(out_fn))


def _print_error_stats_summary(ordered_types, trans_stats, rot_stats):
    print(Fore.YELLOW + "Error stats (mean / median / p95):")
    for k in ordered_types:
        ts = trans_stats[k]
        rs = rot_stats[k]
        print(" - {}: trans {} / {} / {}; rot {} / {} / {}".format(
            k,
            _fmt_stat(ts['mean']),
            _fmt_stat(ts['median']),
            _fmt_stat(ts['p95']),
            _fmt_stat(rs['mean']),
            _fmt_stat(rs['median']),
            _fmt_stat(rs['p95']),
        ))

def _print_error_stats_summary_penalized(ordered_types, trans_stats, rot_stats):
    print(Fore.YELLOW + "Penalized error stats (mean / median / p95):")
    for k in ordered_types:
        ts = trans_stats[k]
        rs = rot_stats[k]
        print(" - {}: trans {} / {} / {}; rot {} / {} / {}".format(
            k,
            _fmt_stat(ts['mean']),
            _fmt_stat(ts['median']),
            _fmt_stat(ts['p95']),
            _fmt_stat(rs['mean']),
            _fmt_stat(rs['median']),
            _fmt_stat(rs['p95']),
        ))

def _print_compare_vs_optimized_summary(ordered_types, trans_stats, rot_stats):
    ref_type = 'optimized_path_yaw'
    if ref_type not in trans_stats or ref_type not in rot_stats:
        return
    print(Fore.YELLOW + "Comparison vs {} (mean diff / ratio):".format(ref_type))
    ref_t = trans_stats[ref_type]['mean']
    ref_r = rot_stats[ref_type]['mean']
    for k in ordered_types:
        if k == ref_type:
            continue
        t = trans_stats[k]['mean']
        r = rot_stats[k]['mean']
        t_diff = t - ref_t if math.isfinite(t) and math.isfinite(ref_t) else float('nan')
        r_diff = r - ref_r if math.isfinite(r) and math.isfinite(ref_r) else float('nan')
        t_ratio = t / ref_t if math.isfinite(t) and math.isfinite(ref_t) and ref_t != 0.0 else float('nan')
        r_ratio = r / ref_r if math.isfinite(r) and math.isfinite(ref_r) and ref_r != 0.0 else float('nan')
        print(" - {}: trans {} / {}; rot {} / {}".format(
            k,
            _fmt_stat(t_diff),
            _fmt_stat(t_ratio),
            _fmt_stat(r_diff),
            _fmt_stat(r_ratio),
        ))


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
        along_path_dir = os.path.join(path, 'along_path')
        if os.path.isdir(along_path_dir):
            along_name = name + '_along_path'
            if along_name not in subdir_map:
                subdir_map[along_name] = along_path_dir
    return subdir_map


def _hasPathYawResults(top_dir):
    for root, _, files in os.walk(top_dir):
        if os.path.basename(root) != 'optimized_path_yaw':
            continue
        pose_err = None
        if pose_e_path_yaw_nm in files:
            pose_err = os.path.join(root, pose_e_path_yaw_nm)
        elif pose_e_nm in files:
            pose_err = os.path.join(root, pose_e_nm)
        if pose_err is None:
            continue
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
        if os.path.basename(root) != 'optimized_path_yaw':
            continue
        err_fn = None
        if pose_e_path_yaw_nm in files:
            err_fn = os.path.join(root, pose_e_path_yaw_nm)
        elif pose_e_nm in files:
            err_fn = os.path.join(root, pose_e_nm)
        if err_fn is None:
            continue
        if not _poseErrorHasData(err_fn):
            continue
        _, trans_e_i, rot_e_i = _loadPoseError(err_fn, max_trans, max_rot)
        acc_trans_e.setdefault('optimized_path_yaw', []).extend(trans_e_i)
        acc_rot_e.setdefault('optimized_path_yaw', []).extend(rot_e_i)

def _collectOptimizedPathYawForCfg(cfg_dir, base_cfg):
    if not base_cfg:
        return [], []
    max_trans = base_cfg.get('max_trans_e_m', base_cfg.get('hist_max_trans_e', float('nan')))
    max_rot = base_cfg.get('max_rot_e_deg', base_cfg.get('hist_max_rot_e', float('nan')))
    trans_all = []
    rot_all = []
    for root, _, files in os.walk(cfg_dir):
        if os.path.basename(root) != 'optimized_path_yaw':
            continue
        err_fn = None
        if pose_e_path_yaw_nm in files:
            err_fn = os.path.join(root, pose_e_path_yaw_nm)
        elif pose_e_nm in files:
            err_fn = os.path.join(root, pose_e_nm)
        if err_fn is None:
            continue
        if not _poseErrorHasData(err_fn):
            continue
        _, trans_e_i, rot_e_i = _loadPoseError(err_fn, max_trans, max_rot)
        trans_all.extend(trans_e_i)
        rot_all.extend(rot_e_i)
    return trans_all, rot_all


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
        pose_e_f_i, twc_f_i = _select_pose_error_file(subdir, pose_e_name, twc_name)
        if not _poseErrorHasData(pose_e_f_i):
            print(Fore.YELLOW + "Skip {} (missing or empty {}).".format(
                subdir, os.path.basename(pose_e_f_i)))
            continue
        valid_entries.append((nm, subdir, pose_e_f_i, twc_f_i))
    if not valid_entries:
        print(Fore.YELLOW + "No pose errors found under {}; skipping.".format(cfg_dir))
        return {}, {}
    subdir_nms = [v[0] for v in valid_entries]
    subdirs = [v[1] for v in valid_entries]
    pose_e_fns = [v[2] for v in valid_entries]
    twc_fns = [v[3] for v in valid_entries]
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
        times_i, _ = _loadPoses(twc_fns[idx])

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
    _save_png(fig, errors_plot_fn, pad=0.25, dpi=300)

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
        fig_reg = plt.figure(figsize=(11, 5))
        ax = fig_reg.add_subplot(111)
        bars = ax.bar(range(len(labels)), ratios, color='gray')
        ax.set_ylabel('non-registered ratio')
        ax.set_ylim(0, 1.05)
        _annotate_bar_values(ax, bars)
        _finalize_reg_bar_plot(fig_reg, ax, labels)
        reg_plot_fn = os.path.join(cfg_dir, 'registration_stats{}.png'.format(plot_suffix))
        _save_png(fig_reg, reg_plot_fn, pad=0.25, dpi=300)

    trans_e_raw = {}
    rot_e_raw = {}
    per_subdir_raw = {}
    for idx, nm in enumerate(subdir_nms):
        if os.path.exists(pose_e_fns[idx]):
            _, trans_e_i_raw, rot_e_i_raw = _loadPoseError(pose_e_fns[idx])
        else:
            trans_e_i_raw = []
            rot_e_i_raw = []

        trans_e_raw[ana_cfg['types'][nm]] = trans_e_i_raw
        rot_e_raw[ana_cfg['types'][nm]] = rot_e_i_raw
        per_subdir_raw[nm] = (trans_e_i_raw, rot_e_i_raw)

    print(Fore.GREEN + '< Done.')

    # write per-variation metrics summary (ATE/RPE if available)
    summary_fn = os.path.join(cfg_dir, 'pose_metrics_summary.csv')
    summary_pen_fn = os.path.join(cfg_dir, 'pose_metrics_summary_penalized.csv')
    penalty_trans = ana_cfg.get('max_trans_e_m', base_cfg.get('hist_max_trans_e', float('nan')) if base_cfg else float('nan'))
    penalty_rot = ana_cfg.get('max_rot_e_deg', base_cfg.get('hist_max_rot_e', float('nan')) if base_cfg else float('nan'))
    with open(summary_fn, 'w') as f:
        f.write(','.join([
            'name', 'type',
            'n_total', 'n_registered', 'registration_rate',
            'ate_rmse', 'ate_mean', 'ate_median', 'ate_p95',
            're_rmse', 're_mean', 're_median', 're_p95',
            'rpe_trans_rmse', 'rpe_rot_rmse', 'rpe_pairs',
            'acc_ratio'
        ]) + '\n')
        for nm in subdir_nms:
            type_i = ana_cfg['types'][nm]
            trans_raw, rot_raw = per_subdir_raw.get(nm, ([], []))
            trans_stats = _summarize_errors(trans_raw)
            rot_stats = _summarize_errors(rot_raw)
            total = trans_stats.get('count', 0) + trans_stats.get('nan_count', 0)
            reg_rate = None
            if total > 0:
                reg_rate = trans_stats.get('count', 0) / total

            metrics_path = os.path.join(subdir_map[nm], 'pose_metrics.json')
            metrics = _read_pose_metrics(metrics_path)
            n_total = metrics.get('n_total') if metrics else total
            n_registered = metrics.get('n_registered') if metrics else trans_stats.get('count', 0)
            reg_rate = metrics.get('registration_rate') if metrics and metrics.get('registration_rate') is not None else reg_rate

            ate_rmse = _get_nested(metrics, 'ate', 'rmse') if metrics else trans_stats.get('rmse')
            ate_mean = _get_nested(metrics, 'ate', 'mean') if metrics else trans_stats.get('mean')
            ate_median = _get_nested(metrics, 'ate', 'median') if metrics else trans_stats.get('median')
            ate_p95 = _get_nested(metrics, 'ate', 'p95') if metrics else trans_stats.get('p95')

            re_rmse = _get_nested(metrics, 're', 'rmse') if metrics else rot_stats.get('rmse')
            re_mean = _get_nested(metrics, 're', 'mean') if metrics else rot_stats.get('mean')
            re_median = _get_nested(metrics, 're', 'median') if metrics else rot_stats.get('median')
            re_p95 = _get_nested(metrics, 're', 'p95') if metrics else rot_stats.get('p95')

            rpe_trans_rmse = _get_nested(metrics, 'rpe', 'trans', 'rmse') if metrics else None
            rpe_rot_rmse = _get_nested(metrics, 'rpe', 'rot', 'rmse') if metrics else None
            rpe_pairs = _get_nested(metrics, 'rpe', 'n_pairs') if metrics else None

            acc_ratio = _get_nested(metrics, 'acc', 'ratio') if metrics else None

            f.write(','.join([
                str(nm),
                str(type_i),
                '' if n_total is None else str(n_total),
                '' if n_registered is None else str(n_registered),
                '' if reg_rate is None else f"{reg_rate:.6f}",
                '' if ate_rmse is None else f"{ate_rmse:.6f}",
                '' if ate_mean is None else f"{ate_mean:.6f}",
                '' if ate_median is None else f"{ate_median:.6f}",
                '' if ate_p95 is None else f"{ate_p95:.6f}",
                '' if re_rmse is None else f"{re_rmse:.6f}",
                '' if re_mean is None else f"{re_mean:.6f}",
                '' if re_median is None else f"{re_median:.6f}",
                '' if re_p95 is None else f"{re_p95:.6f}",
                '' if rpe_trans_rmse is None else f"{rpe_trans_rmse:.6f}",
                '' if rpe_rot_rmse is None else f"{rpe_rot_rmse:.6f}",
                '' if rpe_pairs is None else str(rpe_pairs),
                '' if acc_ratio is None else f"{acc_ratio:.6f}",
            ]) + '\n')

    with open(summary_pen_fn, 'w') as f:
        f.write(','.join([
            'name', 'type',
            'n_total', 'n_registered', 'registration_rate',
            'ate_rmse', 'ate_mean', 'ate_median', 'ate_p95',
            're_rmse', 're_mean', 're_median', 're_p95',
            'ate_rmse_pen', 'ate_mean_pen', 'ate_median_pen', 'ate_p95_pen',
            're_rmse_pen', 're_mean_pen', 're_median_pen', 're_p95_pen',
            'penalty_trans', 'penalty_rot',
            'rpe_trans_rmse', 'rpe_rot_rmse', 'rpe_pairs',
            'acc_ratio'
        ]) + '\n')
        for nm in subdir_nms:
            type_i = ana_cfg['types'][nm]
            trans_raw, rot_raw = per_subdir_raw.get(nm, ([], []))
            trans_stats = _summarize_errors(trans_raw)
            rot_stats = _summarize_errors(rot_raw)
            trans_stats_pen = _stats_with_defaults_penalized(trans_raw, penalty_trans)
            rot_stats_pen = _stats_with_defaults_penalized(rot_raw, penalty_rot)
            total = trans_stats.get('count', 0) + trans_stats.get('nan_count', 0)
            reg_rate = None
            if total > 0:
                reg_rate = trans_stats.get('count', 0) / total

            metrics_path = os.path.join(subdir_map[nm], 'pose_metrics.json')
            metrics = _read_pose_metrics(metrics_path)
            n_total = metrics.get('n_total') if metrics else total
            n_registered = metrics.get('n_registered') if metrics else trans_stats.get('count', 0)
            reg_rate = metrics.get('registration_rate') if metrics and metrics.get('registration_rate') is not None else reg_rate

            ate_rmse = _get_nested(metrics, 'ate', 'rmse') if metrics else trans_stats.get('rmse')
            ate_mean = _get_nested(metrics, 'ate', 'mean') if metrics else trans_stats.get('mean')
            ate_median = _get_nested(metrics, 'ate', 'median') if metrics else trans_stats.get('median')
            ate_p95 = _get_nested(metrics, 'ate', 'p95') if metrics else trans_stats.get('p95')

            re_rmse = _get_nested(metrics, 're', 'rmse') if metrics else rot_stats.get('rmse')
            re_mean = _get_nested(metrics, 're', 'mean') if metrics else rot_stats.get('mean')
            re_median = _get_nested(metrics, 're', 'median') if metrics else rot_stats.get('median')
            re_p95 = _get_nested(metrics, 're', 'p95') if metrics else rot_stats.get('p95')

            rpe_trans_rmse = _get_nested(metrics, 'rpe', 'trans', 'rmse') if metrics else None
            rpe_rot_rmse = _get_nested(metrics, 'rpe', 'rot', 'rmse') if metrics else None
            rpe_pairs = _get_nested(metrics, 'rpe', 'n_pairs') if metrics else None

            acc_ratio = _get_nested(metrics, 'acc', 'ratio') if metrics else None

            f.write(','.join([
                str(nm),
                str(type_i),
                '' if n_total is None else str(n_total),
                '' if n_registered is None else str(n_registered),
                '' if reg_rate is None else f"{reg_rate:.6f}",
                '' if ate_rmse is None else f"{ate_rmse:.6f}",
                '' if ate_mean is None else f"{ate_mean:.6f}",
                '' if ate_median is None else f"{ate_median:.6f}",
                '' if ate_p95 is None else f"{ate_p95:.6f}",
                '' if re_rmse is None else f"{re_rmse:.6f}",
                '' if re_mean is None else f"{re_mean:.6f}",
                '' if re_median is None else f"{re_median:.6f}",
                '' if re_p95 is None else f"{re_p95:.6f}",
                _fmt_stat(trans_stats_pen.get('rmse', float('nan'))),
                _fmt_stat(trans_stats_pen.get('mean', float('nan'))),
                _fmt_stat(trans_stats_pen.get('median', float('nan'))),
                _fmt_stat(trans_stats_pen.get('p95', float('nan'))),
                _fmt_stat(rot_stats_pen.get('rmse', float('nan'))),
                _fmt_stat(rot_stats_pen.get('mean', float('nan'))),
                _fmt_stat(rot_stats_pen.get('median', float('nan'))),
                _fmt_stat(rot_stats_pen.get('p95', float('nan'))),
                _fmt_stat(trans_stats_pen.get('penalty', float('nan'))),
                _fmt_stat(rot_stats_pen.get('penalty', float('nan'))),
                '' if rpe_trans_rmse is None else f"{rpe_trans_rmse:.6f}",
                '' if rpe_rot_rmse is None else f"{rpe_rot_rmse:.6f}",
                '' if rpe_pairs is None else str(rpe_pairs),
                '' if acc_ratio is None else f"{acc_ratio:.6f}",
            ]) + '\n')

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
    per_cfg_stats = {}
    for cfg_d_i in cfg_dirs:
        cfg_key = os.path.basename(cfg_d_i.rstrip(os.sep))
        trans_e_i, rot_e_i = analyzeSingleCfg(
            cfg_d_i, base_cfg=base_ana_cfg, wandb_mod=wandb_mod, wandb_prefix=cfg_key,
            pose_e_name=pose_e_name, twc_name=twc_name, plot_suffix=plot_suffix)
        if base_ana_cfg:
            t_py, r_py = _collectOptimizedPathYawForCfg(cfg_d_i, base_ana_cfg)
            if t_py or r_py:
                trans_e_i = dict(trans_e_i)
                rot_e_i = dict(rot_e_i)
                trans_e_i['optimized_path_yaw'] = t_py
                rot_e_i['optimized_path_yaw'] = r_py
        per_cfg_stats[cfg_key] = (trans_e_i, rot_e_i)
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

    ordered_types = base_ana_cfg.get('ordered_types')
    if not ordered_types:
        if 'labels' in base_ana_cfg:
            ordered_types = list(base_ana_cfg['labels'].keys())
        else:
            ordered_types = sorted(set(acc_trans_e.keys()) | set(acc_rot_e.keys()))
    mode_dirs = _ensure_mode_dirs(top_dir)
    plot_max_trans_e = 1.2 * base_ana_cfg['hist_max_trans_e']
    plot_max_rot_e = 1.2 * base_ana_cfg['hist_max_rot_e']

    trans_stats = {k: _stats_with_defaults(acc_trans_e.get(k, [])) for k in ordered_types}
    rot_stats = {k: _stats_with_defaults(acc_rot_e.get(k, [])) for k in ordered_types}
    trans_stats_orig = {k: _stats_with_defaults_penalized(acc_trans_e.get(k, []), plot_max_trans_e)
                        for k in ordered_types}
    rot_stats_orig = {k: _stats_with_defaults_penalized(acc_rot_e.get(k, []), plot_max_rot_e)
                      for k in ordered_types}

    penalty_trans = base_ana_cfg.get('max_trans_e_m', base_ana_cfg.get('hist_max_trans_e', float('nan')))
    penalty_rot = base_ana_cfg.get('max_rot_e_deg', base_ana_cfg.get('hist_max_rot_e', float('nan')))
    trans_stats_pen = {k: _stats_with_defaults_penalized(acc_trans_e.get(k, []), penalty_trans)
                       for k in ordered_types}
    rot_stats_pen = {k: _stats_with_defaults_penalized(acc_rot_e.get(k, []), penalty_rot)
                     for k in ordered_types}

    stats_by_mode = {
        'finite': (trans_stats, rot_stats),
        'original': (trans_stats_orig, rot_stats_orig),
        'penalized': (trans_stats_pen, rot_stats_pen),
    }

    _write_error_stats_csv(top_dir, plot_suffix, ordered_types, trans_stats, rot_stats)
    _write_error_stats_penalized_csv(top_dir, plot_suffix, ordered_types, trans_stats_pen, rot_stats_pen)
    _write_compare_vs_optimized(top_dir, plot_suffix, ordered_types, trans_stats, rot_stats)
    _print_error_stats_summary(ordered_types, trans_stats, rot_stats)
    _print_error_stats_summary_penalized(ordered_types, trans_stats_pen, rot_stats_pen)
    _print_compare_vs_optimized_summary(ordered_types, trans_stats, rot_stats)

    for mode, (t_stats_m, r_stats_m) in stats_by_mode.items():
        out_dir = mode_dirs[mode]
        if mode == 'finite':
            _write_error_stats_csv(out_dir, plot_suffix, ordered_types, t_stats_m, r_stats_m)
        else:
            _write_error_stats_penalized_csv(out_dir, plot_suffix, ordered_types, t_stats_m, r_stats_m)
        _write_compare_vs_optimized(out_dir, plot_suffix, ordered_types, t_stats_m, r_stats_m)

    # Per-config stats (nan %, mean/std) across variations.
    per_cfg_map_by_mode = {m: {} for m in ('finite', 'original', 'penalized')}
    for cfg_name in cfg_nms:
        trans_e_i, rot_e_i = per_cfg_stats.get(cfg_name, ({}, {}))
        for v in ordered_types:
            t_vals = trans_e_i.get(v, [])
            r_vals = rot_e_i.get(v, [])
            total = len(r_vals)
            if total > 0:
                nan_count = int(np.sum(np.isnan(np.asarray(r_vals, dtype=float))))
                nan_ratio = nan_count / total
            else:
                nan_count = 0
                nan_ratio = float('nan')
            t_stats_f = _finite_stats(t_vals)
            r_stats_f = _finite_stats(r_vals)
            t_stats_o = _stats_with_defaults_penalized(t_vals, plot_max_trans_e)
            r_stats_o = _stats_with_defaults_penalized(r_vals, plot_max_rot_e)
            t_stats_p = _stats_with_defaults_penalized(t_vals, penalty_trans)
            r_stats_p = _stats_with_defaults_penalized(r_vals, penalty_rot)

            per_cfg_map_by_mode['finite'][(cfg_name, v)] = {
                'cfg': cfg_name,
                'type': v,
                'nan_ratio': nan_ratio,
                'nan_count': nan_count,
                'total': total,
                'trans_mean': t_stats_f['mean'],
                'trans_std': t_stats_f['std'],
                'trans_count': t_stats_f['count'],
                'rot_mean': r_stats_f['mean'],
                'rot_std': r_stats_f['std'],
                'rot_count': r_stats_f['count'],
            }
            per_cfg_map_by_mode['original'][(cfg_name, v)] = {
                'cfg': cfg_name,
                'type': v,
                'nan_ratio': nan_ratio,
                'nan_count': nan_count,
                'total': total,
                'trans_mean': t_stats_o['mean'],
                'trans_std': t_stats_o.get('std', float('nan')),
                'trans_count': t_stats_o['count'],
                'rot_mean': r_stats_o['mean'],
                'rot_std': r_stats_o.get('std', float('nan')),
                'rot_count': r_stats_o['count'],
            }
            per_cfg_map_by_mode['penalized'][(cfg_name, v)] = {
                'cfg': cfg_name,
                'type': v,
                'nan_ratio': nan_ratio,
                'nan_count': nan_count,
                'total': total,
                'trans_mean': t_stats_p['mean'],
                'trans_std': t_stats_p.get('std', float('nan')),
                'trans_count': t_stats_p['count'],
                'rot_mean': r_stats_p['mean'],
                'rot_std': r_stats_p.get('std', float('nan')),
                'rot_count': r_stats_p['count'],
            }

    per_cfg_fn = os.path.join(top_dir, 'overall_per_cfg_stats{}.csv'.format(plot_suffix))
    with open(per_cfg_fn, 'w') as f:
        f.write('cfg,type,nan_ratio,nan_count,total,trans_mean,trans_std,trans_count,rot_mean,rot_std,rot_count\n')
        for v in ordered_types:
            for cfg_name in cfg_nms:
                row = per_cfg_map_by_mode['finite'].get((cfg_name, v))
                if row is None:
                    row = {
                        'cfg': cfg_name,
                        'type': v,
                        'nan_ratio': float('nan'),
                        'nan_count': 0,
                        'total': 0,
                        'trans_mean': float('nan'),
                        'trans_std': float('nan'),
                        'trans_count': 0,
                        'rot_mean': float('nan'),
                        'rot_std': float('nan'),
                        'rot_count': 0,
                    }
                f.write('{},{},{},{},{},{},{},{},{},{},{}\n'.format(
                    row['cfg'],
                    row['type'],
                    _fmt_stat(row['nan_ratio']),
                    row['nan_count'],
                    row['total'],
                    _fmt_stat(row['trans_mean']),
                    _fmt_stat(row['trans_std']),
                    row['trans_count'],
                    _fmt_stat(row['rot_mean']),
                    _fmt_stat(row['rot_std']),
                    row['rot_count'],
                ))
    print(Fore.YELLOW + "Saved per-config stats to {}".format(per_cfg_fn))

    for mode, per_map in per_cfg_map_by_mode.items():
        out_dir = mode_dirs[mode]
        out_fn = os.path.join(out_dir, 'overall_per_cfg_stats{}.csv'.format(plot_suffix))
        with open(out_fn, 'w') as f:
            f.write('cfg,type,nan_ratio,nan_count,total,trans_mean,trans_std,trans_count,rot_mean,rot_std,rot_count\n')
            for v in ordered_types:
                for cfg_name in cfg_nms:
                    row = per_map.get((cfg_name, v))
                    if row is None:
                        row = {
                            'cfg': cfg_name,
                            'type': v,
                            'nan_ratio': float('nan'),
                            'nan_count': 0,
                            'total': 0,
                            'trans_mean': float('nan'),
                            'trans_std': float('nan'),
                            'trans_count': 0,
                            'rot_mean': float('nan'),
                            'rot_std': float('nan'),
                            'rot_count': 0,
                        }
                    f.write('{},{},{},{},{},{},{},{},{},{},{}\n'.format(
                        row['cfg'],
                        row['type'],
                        _fmt_stat(row['nan_ratio']),
                        row['nan_count'],
                        row['total'],
                        _fmt_stat(row['trans_mean']),
                        _fmt_stat(row['trans_std']),
                        row['trans_count'],
                        _fmt_stat(row['rot_mean']),
                        _fmt_stat(row['rot_std']),
                        row['rot_count'],
                    ))
        print(Fore.YELLOW + "Saved per-config stats to {}".format(out_fn))

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

    reg_labels = []
    reg_values = []
    for k in ordered_types:
        if k not in reg_overall:
            continue
        reg_labels.append(k)
        reg_values.append(reg_overall[k][0])

    under_trans_thresh = 1.0
    under_rot_thresh = 10.0

    mode_out_dirs = {
        'finite': [mode_dirs['finite']],
        'original': [mode_dirs['original'], top_dir],
        'penalized': [mode_dirs['penalized']],
    }

    if ordered_types:
        labels = ordered_types
        x = np.arange(len(labels))
        width = 0.38
        trans_labels = [base_ana_cfg['labels'][v] for v in ordered_types]
        trans_colors = [base_ana_cfg['colors'][v] for v in ordered_types]
        rot_labels = [base_ana_cfg['labels'][v] for v in ordered_types]
        rot_colors = [base_ana_cfg['colors'][v] for v in ordered_types]

        for mode in ('finite', 'original', 'penalized'):
            out_dirs = mode_out_dirs[mode]
            t_stats_m, r_stats_m = stats_by_mode[mode]
            include_nan = mode != 'finite'

            for out_dir in out_dirs:
                # Registration stats (same content for all modes).
                reg_fn = os.path.join(out_dir, 'registration_stats{}.csv'.format(plot_suffix))
                with open(reg_fn, 'w') as f:
                    f.write('type,non_registered_ratio,registered_ratio,n_failed,n_total\n')
                    for k in sorted(reg_overall.keys()):
                        nonreg_ratio, reg_ratio, n_failed, total = reg_overall[k]
                        if math.isnan(nonreg_ratio):
                            f.write("{},nan,nan,{},{}\n".format(k, n_failed, total))
                        else:
                            f.write("{},{:.4f},{:.4f},{},{}\n".format(
                                k, nonreg_ratio, reg_ratio, n_failed, total))

                if reg_labels:
                    fig_reg = plt.figure(figsize=(11, 5))
                    ax = fig_reg.add_subplot(111)
                    bars = ax.bar(range(len(reg_labels)), reg_values, color='gray')
                    ax.set_ylabel('non-registered ratio')
                    ax.set_ylim(0, 1.05)
                    _annotate_bar_values(ax, bars)
                    _finalize_reg_bar_plot(fig_reg, ax, reg_labels)
                    reg_plot_fn = os.path.join(out_dir, 'registration_stats{}.png'.format(plot_suffix))
                    _save_png(fig_reg, reg_plot_fn, pad=0.25, dpi=300)

                # Under-threshold stats.
                under_fn = os.path.join(out_dir, 'overall_under_threshold{}.csv'.format(plot_suffix))
                with open(under_fn, 'w') as f:
                    f.write('type,trans_thresh,rot_thresh,trans_ratio,trans_under,trans_total,rot_ratio,rot_under,rot_total\n')
                    for k in ordered_types:
                        t_ratio, t_under, t_total = _ratio_under_threshold(
                            acc_trans_e.get(k, []), under_trans_thresh, include_nan=include_nan)
                        r_ratio, r_under, r_total = _ratio_under_threshold(
                            acc_rot_e.get(k, []), under_rot_thresh, include_nan=include_nan)
                        f.write('{},{},{},{},{},{},{},{},{}\n'.format(
                            k,
                            _fmt_stat(under_trans_thresh),
                            _fmt_stat(under_rot_thresh),
                            _fmt_stat(t_ratio),
                            t_under,
                            t_total,
                            _fmt_stat(r_ratio),
                            r_under,
                            r_total,
                        ))

                trans_ratios = []
                rot_ratios = []
                for k in labels:
                    t_ratio, _, _ = _ratio_under_threshold(
                        acc_trans_e.get(k, []), under_trans_thresh, include_nan=include_nan)
                    r_ratio, _, _ = _ratio_under_threshold(
                        acc_rot_e.get(k, []), under_rot_thresh, include_nan=include_nan)
                    trans_ratios.append(t_ratio)
                    rot_ratios.append(r_ratio)
                trans_plot = [0.0 if not math.isfinite(v) else v for v in trans_ratios]
                rot_plot = [0.0 if not math.isfinite(v) else v for v in rot_ratios]
                fig_thr = plt.figure(figsize=(12, 5))
                ax_thr = fig_thr.add_subplot(111)
                bars_t = ax_thr.bar(x - width / 2.0, trans_plot, width, label='pos < 1m', color='#4c72b0')
                bars_r = ax_thr.bar(x + width / 2.0, rot_plot, width, label='rot < 10deg', color='#dd8452')
                ax_thr.set_ylabel('ratio under threshold')
                ax_thr.set_ylim(0, 1.05)
                ax_thr.legend(loc='upper right', fontsize='small')
                _annotate_bar_values(ax_thr, bars_t, as_percent=True)
                _annotate_bar_values(ax_thr, bars_r, as_percent=True)
                _finalize_reg_bar_plot(fig_thr, ax_thr, labels)
                thr_plot_fn = os.path.join(out_dir, 'overall_under_threshold{}.png'.format(plot_suffix))
                _save_png(fig_thr, thr_plot_fn, pad=0.25, dpi=300)

                # Max errors.
                max_fn = os.path.join(out_dir, 'overall_max_errors{}.csv'.format(plot_suffix))
                with open(max_fn, 'w') as f:
                    f.write('type,max_trans,max_rot\n')
                    for k in ordered_types:
                        f.write('{},{},{}\n'.format(
                            k,
                            _fmt_stat(t_stats_m[k]['max']),
                            _fmt_stat(r_stats_m[k]['max']),
                        ))

                t_max_plot = [0.0 if not math.isfinite(t_stats_m[k]['max']) else t_stats_m[k]['max'] for k in labels]
                r_max_plot = [0.0 if not math.isfinite(r_stats_m[k]['max']) else r_stats_m[k]['max'] for k in labels]
                fig_max = plt.figure(figsize=(12, 5))
                ax_max = fig_max.add_subplot(111)
                bars_tm = ax_max.bar(x - width / 2.0, t_max_plot, width, label='max pos (m)', color='#4c72b0')
                bars_rm = ax_max.bar(x + width / 2.0, r_max_plot, width, label='max rot (deg)', color='#dd8452')
                ax_max.set_ylabel('max error{}'.format(' (penalized)' if mode == 'penalized' else '' if mode == 'finite' else ' (original)'))
                ax_max.legend(loc='upper right', fontsize='small')
                _annotate_bar_values(ax_max, bars_tm)
                _annotate_bar_values(ax_max, bars_rm)
                _finalize_reg_bar_plot(fig_max, ax_max, labels)
                max_plot_fn = os.path.join(out_dir, 'overall_max_errors{}.png'.format(plot_suffix))
                _save_png(fig_max, max_plot_fn, pad=0.25, dpi=300)

                # Mean/std stats.
                mean_std_fn = os.path.join(out_dir, 'overall_mean_std_errors{}.csv'.format(plot_suffix))
                with open(mean_std_fn, 'w') as f:
                    f.write('type,trans_mean,trans_std,trans_count,rot_mean,rot_std,rot_count\n')
                    for k in ordered_types:
                        f.write('{},{},{},{},{},{},{}\n'.format(
                            k,
                            _fmt_stat(t_stats_m[k]['mean']),
                            _fmt_stat(t_stats_m[k].get('std', float('nan'))),
                            t_stats_m[k]['count'],
                            _fmt_stat(r_stats_m[k]['mean']),
                            _fmt_stat(r_stats_m[k].get('std', float('nan'))),
                            r_stats_m[k]['count'],
                        ))

                fig_ms = plt.figure(figsize=(12, 6))
                axes_ms = fig_ms.subplots(2, 1, sharex=True)
                ax_ms_t, ax_ms_r = axes_ms[0], axes_ms[1]
                t_means = [t_stats_m[k]['mean'] if math.isfinite(t_stats_m[k]['mean']) else 0.0 for k in labels]
                t_stds = [t_stats_m[k].get('std', float('nan')) if math.isfinite(t_stats_m[k].get('std', float('nan'))) else 0.0 for k in labels]
                r_means = [r_stats_m[k]['mean'] if math.isfinite(r_stats_m[k]['mean']) else 0.0 for k in labels]
                r_stds = [r_stats_m[k].get('std', float('nan')) if math.isfinite(r_stats_m[k].get('std', float('nan'))) else 0.0 for k in labels]
                ax_ms_t.bar(x, t_means, yerr=t_stds, color='#4c72b0', capsize=3)
                ax_ms_r.bar(x, r_means, yerr=r_stds, color='#dd8452', capsize=3)
                ax_ms_t.set_ylabel('pos mean ± std{}'.format(' (penalized)' if mode == 'penalized' else '' if mode == 'finite' else ' (original)'))
                ax_ms_r.set_ylabel('rot mean ± std{}'.format(' (penalized)' if mode == 'penalized' else '' if mode == 'finite' else ' (original)'))
                ax_ms_r.set_xlabel('variation')
                _finalize_two_row_plot(fig_ms, ax_ms_r, labels)
                ms_plot_fn = os.path.join(out_dir, 'overall_mean_std_errors{}.png'.format(plot_suffix))
                _save_png(fig_ms, ms_plot_fn, pad=0.25, dpi=300)

                # Table view with numeric values.
                fig_tbl = plt.figure(figsize=(12, max(2.5, 0.35 * len(labels) + 1.5)))
                ax_tbl = fig_tbl.add_subplot(111)
                ax_tbl.axis('off')
                cell_text = []
                for k in labels:
                    cell_text.append([
                        _fmt_stat(t_stats_m[k]['mean']),
                        _fmt_stat(t_stats_m[k].get('std', float('nan'))),
                        _fmt_stat(r_stats_m[k]['mean']),
                        _fmt_stat(r_stats_m[k].get('std', float('nan'))),
                        str(t_stats_m[k]['count']),
                        str(r_stats_m[k]['count']),
                    ])
                col_labels = ['pos_mean', 'pos_std', 'rot_mean', 'rot_std', 'pos_n', 'rot_n']
                table = ax_tbl.table(cellText=cell_text, colLabels=col_labels,
                                     rowLabels=labels, loc='center')
                table.auto_set_font_size(False)
                table.set_fontsize(9)
                table.scale(1.0, 1.2)
                _finalize_table_plot(fig_tbl)
                tbl_plot_fn = os.path.join(out_dir, 'overall_mean_std_table{}.png'.format(plot_suffix))
                _save_png(fig_tbl, tbl_plot_fn, pad=0.25, dpi=300)

                # Overall histogram (CDF) with mode-specific NaN handling.
                trans_errors = [_mode_values(acc_trans_e.get(v, []), mode, plot_max_trans_e, penalty_trans)
                                for v in ordered_types]
                rot_errors = [_mode_values(acc_rot_e.get(v, []), mode, plot_max_rot_e, penalty_rot)
                              for v in ordered_types]

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
                overall_hist_fn = os.path.join(out_dir, 'overall_hist{}.png'.format(plot_suffix))
                _save_png(fig, overall_hist_fn, pad=0.25, dpi=300)

                # Combined per-config plot (nan %, mean/std) across all cfgs and variations.
                per_map = per_cfg_map_by_mode.get(mode, {})
                if per_map:
                    labels_cfg = []
                    nan_vals = []
                    t_means_cfg = []
                    t_stds_cfg = []
                    r_means_cfg = []
                    r_stds_cfg = []
                    for v in ordered_types:
                        for cfg_name in cfg_nms:
                            row = per_map.get((cfg_name, v))
                            if row is None:
                                continue
                            labels_cfg.append(f"{v}/{cfg_name}")
                            nan_vals.append(0.0 if not math.isfinite(row['nan_ratio']) else row['nan_ratio'])
                            t_means_cfg.append(0.0 if not math.isfinite(row['trans_mean']) else row['trans_mean'])
                            t_stds_cfg.append(0.0 if not math.isfinite(row['trans_std']) else row['trans_std'])
                            r_means_cfg.append(0.0 if not math.isfinite(row['rot_mean']) else row['rot_mean'])
                            r_stds_cfg.append(0.0 if not math.isfinite(row['rot_std']) else row['rot_std'])

                    n = len(labels_cfg)
                    if n > 0:
                        fig_cfg = plt.figure(figsize=(max(14, 0.35 * n), 9))
                        axes_cfg = fig_cfg.subplots(3, 1, sharex=True)
                        ax_nan, ax_tm, ax_rm = axes_cfg[0], axes_cfg[1], axes_cfg[2]
                        x_cfg = np.arange(n)
                        ax_nan.bar(x_cfg, nan_vals, color='#7f7f7f')
                        ax_nan.set_ylabel('nan ratio')
                        ax_nan.set_ylim(0, 1.05)

                        ax_tm.bar(x_cfg, t_means_cfg, yerr=t_stds_cfg, color='#4c72b0', capsize=2)
                        ax_tm.set_ylabel('pos mean ± std')

                        ax_rm.bar(x_cfg, r_means_cfg, yerr=r_stds_cfg, color='#dd8452', capsize=2)
                        ax_rm.set_ylabel('rot mean ± std')
                        ax_rm.set_xlabel('cfg / variation')

                        _finalize_long_x_plot(fig_cfg, ax_rm, labels_cfg)
                        per_cfg_plot_fn = os.path.join(out_dir, 'overall_per_cfg_summary{}.png'.format(plot_suffix))
                        _save_png(fig_cfg, per_cfg_plot_fn, pad=0.25, dpi=300)


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
