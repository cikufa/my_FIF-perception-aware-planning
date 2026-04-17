#!/usr/bin/env python3
import argparse
import math
import os
import numpy as np


def load_twc(path):
    samples = []
    with open(path, 'r') as f:
        for line in f:
            if not line.strip() or line.startswith('#'):
                continue
            vals = [float(v) for v in line.split()]
            if len(vals) < 17:
                continue
            t = vals[0]
            Twc = np.eye(4)
            Twc_vals = vals[1:17]
            Twc[:4, :4] = np.array(Twc_vals, dtype=float).reshape(4, 4)
            samples.append((t, Twc))
    return samples


def save_twc(path, samples):
    with open(path, 'w') as f:
        f.write("# transformation matrices - {} entries: time; mat of size: 4 x 4\n".format(
            len(samples)))
        for t, Twc in samples:
            row = [t] + Twc.reshape(-1).tolist()
            f.write(" ".join("{:.17g}".format(v) for v in row) + "\n")


def build_path_yaw(samples):
    if not samples:
        return []
    out = []
    prev_dir = np.array([1.0, 0.0, 0.0])
    for i, (t, Twc) in enumerate(samples):
        pos = Twc[0:3, 3].copy()
        if i + 1 < len(samples):
            dir_vec = samples[i + 1][1][0:3, 3] - pos
        elif i > 0:
            dir_vec = pos - samples[i - 1][1][0:3, 3]
        else:
            dir_vec = prev_dir.copy()
        dir_vec[2] = 0.0
        norm = np.linalg.norm(dir_vec)
        if norm < 1e-6:
            dir_vec = prev_dir.copy()
        else:
            dir_vec = dir_vec / norm
            prev_dir = dir_vec.copy()

        c1 = np.array([0.0, 0.0, -1.0])
        c2 = np.array([dir_vec[0], dir_vec[1], 0.0])
        c0 = np.cross(c1, c2)
        if np.linalg.norm(c0) < 1e-6:
            c0 = np.array([1.0, 0.0, 0.0])
        else:
            c0 = c0 / np.linalg.norm(c0)

        R = np.column_stack([c0, c1, c2])
        Twc_new = np.eye(4)
        Twc_new[0:3, 0:3] = R
        Twc_new[0:3, 3] = pos
        out.append((t, Twc_new))
    return out


def rotmat_to_qvec(R):
    Rxx, Ryx, Rzx, Rxy, Ryy, Rzy, Rxz, Ryz, Rzz = R.flat
    K = np.array([
        [Rxx - Ryy - Rzz, 0, 0, 0],
        [Ryx + Rxy, Ryy - Rxx - Rzz, 0, 0],
        [Rzx + Rxz, Rzy + Ryz, Rzz - Rxx - Ryy, 0],
        [Ryz - Rzy, Rzx - Rxz, Rxy - Ryx, Rxx + Ryy + Rzz]]) / 3.0
    eigvals, eigvecs = np.linalg.eigh(K)
    qvec = eigvecs[[3, 0, 1, 2], np.argmax(eigvals)]
    if qvec[0] < 0:
        qvec *= -1
    return qvec  # [w, x, y, z]


def clamp_axis(angle_deg):
    angle_deg = math.fmod(angle_deg, 360.0)
    if angle_deg < 0.0:
        angle_deg += 360.0
    return angle_deg


def normalize_axis(angle_deg):
    angle_deg = clamp_axis(angle_deg)
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg


def quaternion_to_euler_ue(qw, qx, qy, qz):
    singularity = qz * qx - qw * qy
    yaw_y = 2.0 * (qw * qz + qx * qy)
    yaw_x = 1.0 - 2.0 * (qy * qy + qz * qz)
    threshold = 0.4999995
    rad_to_deg = 180.0 / math.pi
    if singularity < -threshold:
        pitch = -90.0
        yaw = math.atan2(yaw_y, yaw_x) * rad_to_deg
        roll = normalize_axis(-yaw - (2.0 * math.atan2(qx, qw) * rad_to_deg))
    elif singularity > threshold:
        pitch = 90.0
        yaw = math.atan2(yaw_y, yaw_x) * rad_to_deg
        roll = normalize_axis(yaw - (2.0 * math.atan2(qx, qw) * rad_to_deg))
    else:
        pitch = math.asin(2.0 * singularity) * rad_to_deg
        yaw = math.atan2(yaw_y, yaw_x) * rad_to_deg
        roll = math.atan2(-2.0 * (qw * qx + qy * qz),
                          (1.0 - 2.0 * (qx * qx + qy * qy))) * rad_to_deg
    return pitch, yaw, roll


def standard_twc_to_ue(Twc):
    T_wue_w = np.eye(4)
    T_wue_w[1, 1] = -1.0
    T_c_cue = np.zeros((4, 4))
    T_c_cue[0, 1] = 1.0
    T_c_cue[1, 2] = -1.0
    T_c_cue[2, 0] = 1.0
    T_c_cue[3, 3] = 1.0
    return T_wue_w.dot(Twc).dot(T_c_cue)


def save_ue(path, samples):
    with open(path, 'w') as f:
        f.write("# ue poses - {} entries: time; mat of size: 1 x 6\n".format(
            len(samples)))
        for t, Twc in samples:
            Twc_ue = standard_twc_to_ue(Twc)
            R = Twc_ue[0:3, 0:3]
            qvec = rotmat_to_qvec(R)
            qw, qx, qy, qz = qvec
            pitch, yaw, roll = quaternion_to_euler_ue(qw, qx, qy, qz)
            pos = Twc_ue[0:3, 3]
            f.write("{:.17g} {:.17g} {:.17g} {:.17g} {:.17g} {:.17g} {:.17g}\n".format(
                t, pos[0], pos[1], pos[2], pitch, yaw, roll))


def main():
    parser = argparse.ArgumentParser(
        description="Generate stamped_Twc_path_yaw.txt and stamped_Twc_ue_path_yaw.txt from stamped_Twc.txt")
    parser.add_argument('--input_dir', required=True,
                        help='Directory containing stamped_Twc.txt')
    parser.add_argument('--output_dir', default=None,
                        help='Output directory (defaults to input_dir)')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_dir = args.output_dir or input_dir
    twc_in = os.path.join(input_dir, 'stamped_Twc.txt')
    if not os.path.exists(twc_in):
        raise FileNotFoundError("Missing stamped_Twc.txt: {}".format(twc_in))
    os.makedirs(output_dir, exist_ok=True)

    samples = load_twc(twc_in)
    path_samples = build_path_yaw(samples)
    twc_out = os.path.join(output_dir, 'stamped_Twc_path_yaw.txt')
    ue_out = os.path.join(output_dir, 'stamped_Twc_ue_path_yaw.txt')
    save_twc(twc_out, path_samples)
    save_ue(ue_out, path_samples)


if __name__ == '__main__':
    main()
