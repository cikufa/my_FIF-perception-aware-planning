#!/usr/bin/env python3

import os
import argparse
import json
import numpy as np
from colorama import Fore, init

import utils.exp_utils as eu
from utils.colmap_read_model import read_images_binary
import math

init(autoreset=True)

def rotmat_to_euler_ue(R):
    """Convert rotation matrix to Unreal Engine (Pitch, Yaw, Roll) in degrees"""
    # Unreal Engine convention: Pitch (X), Yaw (Z), Roll (Y) ?
    # Assuming Z-up in UE
    sy = -R[2, 0]
    cy = np.sqrt(1 - sy**2)

    if cy > 1e-6:
        pitch = np.arctan2(R[2,1], R[2,2])
        yaw = np.arcsin(sy)
        roll = np.arctan2(R[1,0], R[0,0])
    else:
        pitch = 0
        yaw = np.arcsin(sy)
        roll = np.arctan2(-R[0,1], R[1,1])
    
    return np.rad2deg([pitch, yaw, roll])

def _rot_err_deg_from_R(R):
    """Return rotation error (deg) from a rotation matrix."""
    val = 0.5 * (np.trace(R) - 1.0)
    val = float(np.clip(val, -1.0, 1.0))
    return float(np.rad2deg(np.arccos(val)))


def _stats(values):
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {}
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
        "rmse": float(np.sqrt(np.mean(arr ** 2))),
    }


def _relative_pose_error(Twc_gt_prev, Twc_gt_curr, Twc_est_prev, Twc_est_curr):
    T_rel_gt = np.linalg.inv(Twc_gt_prev) @ Twc_gt_curr
    T_rel_est = np.linalg.inv(Twc_est_prev) @ Twc_est_curr
    T_err = np.linalg.inv(T_rel_gt) @ T_rel_est
    trans_err = float(np.linalg.norm(T_err[0:3, 3]))
    rot_err = _rot_err_deg_from_R(T_err[0:3, 0:3])
    return trans_err, rot_err


def _accuracy_ratio(te_list, re_list, t_thresh, r_thresh):
    if t_thresh is None or r_thresh is None:
        return None
    total = 0
    passed = 0
    for te, re in zip(te_list, re_list):
        if not (math.isfinite(te) and math.isfinite(re)):
            continue
        total += 1
        if te <= t_thresh and re <= r_thresh:
            passed += 1
    if total == 0:
        return {"ratio": None, "count": 0, "total": 0}
    return {"ratio": float(passed / total), "count": int(passed), "total": int(total)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Calculate registeraition error")
    parser.add_argument('--reg_model_dir', required=True, help='model folder containing all images')
    parser.add_argument('--reg_img_name_to_colmap_Tcw', required=True,
                        help='groundtruth poses, colmap format')
    parser.add_argument('--reg_img_dir', required=True, help='used to compute the relative path')
    parser.add_argument('--output_path', required=True, help='where to write the pose errors')
    parser.add_argument('--metrics_out', default='',
                        help='optional metrics output file (default: <output_path>/pose_metrics.json)')
    parser.add_argument('--acc_trans_thresh', type=float, default=None,
                        help='translation threshold (m) for accuracy ratio')
    parser.add_argument('--acc_rot_thresh', type=float, default=None,
                        help='rotation threshold (deg) for accuracy ratio')
    args = parser.parse_args()

    base_img_dir = os.path.abspath(os.path.join(args.reg_model_dir, '../images'))
    print(Fore.GREEN + "Base image folder is {}".format(base_img_dir))
    rel_path = os.path.relpath(args.reg_img_dir, base_img_dir)

    reg_names, reg_qtvec = eu.readNamedValues(args.reg_img_name_to_colmap_Tcw)
    registered_images = read_images_binary(os.path.join(args.reg_model_dir, 'images.bin'))
    print(Fore.GREEN + "Read {} images".format(len(registered_images)))
    reg_img_nm_to_id = {}
    for k in registered_images:
        reg_img_nm_to_id[registered_images[k].name] = k

    out_fn = os.path.join(args.output_path, "pose_errors.txt")
    mis_cnt = 0
    ue_poses_list = []
    te_list = []
    re_list = []
    gt_Twc_list = []
    est_Twc_list = []
    reg_indices = []
    with open(out_fn, 'w') as f:
        f.write("# image_name trans_e_m rot_e_deg\n")
        for idx, img_nm in enumerate(reg_names):
            Twc_gt = eu.colmapQtToTwc(reg_qtvec[idx][0:4], reg_qtvec[idx][4:7])
            rel_img_path = os.path.join(rel_path, img_nm)
            te = float('nan')
            re = float('nan')
            if rel_img_path in reg_img_nm_to_id:
                Twc_reg = registered_images[reg_img_nm_to_id[rel_img_path]].Twc()
                # print("TWC gt\n",Twc_gt, "\nTWC reg\n",Twc_reg)
                te, re = eu.calPoseError(Twc_gt, Twc_reg)

                # Convert Twc_reg to UE format
                pos = Twc_reg[0:3, 3]  # XYZ
                R = Twc_reg[0:3, 0:3]
                pitch, yaw, roll = rotmat_to_euler_ue(R)
                Twc_reg_ue = [*pos, pitch, yaw, roll]
                ue_poses_list.append((img_nm, Twc_reg_ue))
                gt_Twc_list.append(Twc_gt)
                est_Twc_list.append(Twc_reg)
                reg_indices.append(idx)
            else:
                mis_cnt += 1
            te_list.append(te)
            re_list.append(re)
            f.write('{} {} {} {} {}\n'.format(img_nm, te, re, mis_cnt, len(ue_poses_list)))


    ue_output_fn = os.path.join(args.output_path, "registered_poses_ue.txt")
    with open(ue_output_fn, 'w') as ue_f:
        for img_nm, pose in ue_poses_list:
            ue_f.write(f"{img_nm} {' '.join(map(str, pose))}\n")

    print(Fore.GREEN + "{} out of {} images failed to registers.".format(mis_cnt, len(reg_names)))

    # --- summary metrics ---
    n_total = len(te_list)
    n_registered = len(est_Twc_list)
    reg_rate = float(n_registered / n_total) if n_total > 0 else None

    ate_stats = _stats(te_list)
    re_stats = _stats(re_list)

    rpe_trans = []
    rpe_rot = []
    rpe_gaps = []
    for i in range(1, len(est_Twc_list)):
        prev_idx = reg_indices[i - 1]
        cur_idx = reg_indices[i]
        rpe_gaps.append(cur_idx - prev_idx)
        trans_err, rot_err = _relative_pose_error(
            gt_Twc_list[i - 1], gt_Twc_list[i],
            est_Twc_list[i - 1], est_Twc_list[i])
        rpe_trans.append(trans_err)
        rpe_rot.append(rot_err)

    rpe_stats_trans = _stats(rpe_trans)
    rpe_stats_rot = _stats(rpe_rot)
    rpe_info = {
        "trans": rpe_stats_trans,
        "rot": rpe_stats_rot,
        "n_pairs": int(len(rpe_trans)),
        "mean_gap": float(np.mean(rpe_gaps)) if rpe_gaps else None,
        "max_gap": int(np.max(rpe_gaps)) if rpe_gaps else None,
    }

    acc_info = _accuracy_ratio(te_list, re_list, args.acc_trans_thresh, args.acc_rot_thresh)
    if acc_info is not None:
        acc_info["trans_thresh"] = args.acc_trans_thresh
        acc_info["rot_thresh"] = args.acc_rot_thresh

    metrics = {
        "n_total": int(n_total),
        "n_registered": int(n_registered),
        "registration_rate": reg_rate,
        "ate": ate_stats,
        "re": re_stats,
        "rpe": rpe_info,
    }
    if acc_info is not None:
        metrics["acc"] = acc_info

    metrics_out = args.metrics_out
    if not metrics_out:
        metrics_out = os.path.join(args.output_path, "pose_metrics.json")
    elif os.path.isdir(metrics_out):
        metrics_out = os.path.join(metrics_out, "pose_metrics.json")

    try:
        with open(metrics_out, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(Fore.GREEN + f"Wrote metrics: {metrics_out}")
    except Exception as e:
        print(Fore.YELLOW + f"Failed to write metrics ({metrics_out}): {e}")
