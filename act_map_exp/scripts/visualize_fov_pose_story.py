#!/usr/bin/env python2
"""Static RViz view of FoV optimization progression for one pose on a trajectory."""

from __future__ import print_function

import argparse
import os

import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def make_color(r, g, b, a):
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def np_to_point(xyz):
    p = Point()
    p.x = float(xyz[0])
    p.y = float(xyz[1])
    p.z = float(xyz[2])
    return p


def marker_base(frame, ns, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def parse_quiver_blocks(path, min_cols=6):
    blocks = []
    cur = []
    with open(path, "r") as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("#"):
                continue
            if not line:
                if cur:
                    blocks.append(np.asarray(cur, dtype=float))
                    cur = []
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) >= min_cols:
                cur.append([float(v) for v in parts[:min_cols]])
    if cur:
        blocks.append(np.asarray(cur, dtype=float))

    if not blocks:
        raise RuntimeError("No iteration blocks found in {}".format(path))

    min_n_pose = min(len(b) for b in blocks)
    blocks = [b[:min_n_pose] for b in blocks]
    return np.stack(blocks, axis=0)


def parse_cols(cols):
    parts = [p.strip() for p in cols.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError("--points-cols must be 3 comma-separated integers")
    return int(parts[0]), int(parts[1]), int(parts[2])


def append_suffix_before_extension(path, suffix):
    base, ext = os.path.splitext(path)
    return base + suffix + ext


def load_points(path, cols, stride, max_points):
    pts = []
    with open(path, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) <= max(cols):
                continue
            try:
                pts.append([float(parts[cols[0]]), float(parts[cols[1]]), float(parts[cols[2]])])
            except ValueError:
                continue
    if not pts:
        return np.empty((0, 3), dtype=float)
    arr = np.asarray(pts, dtype=float)
    if stride > 1:
        arr = arr[::stride]
    if max_points > 0 and arr.shape[0] > max_points:
        rng = np.random.RandomState(7)
        idx = rng.choice(arr.shape[0], size=max_points, replace=False)
        arr = arr[idx]
    return arr


def interp_red_green(t, alpha):
    t = max(0.0, min(1.0, t))
    return make_color(1.0 - t, 0.1 + 0.9 * t, 0.1, alpha)


def compute_visible_indices(pos, direction, points, cos_fov, min_range, max_range):
    if points.size == 0:
        return np.empty((0,), dtype=int)
    v = points - pos[None, :]
    dist = np.linalg.norm(v, axis=1) + 1e-9
    mask = np.ones_like(dist, dtype=bool)
    if max_range > 0:
        mask &= dist <= max_range
    if min_range > 0:
        mask &= dist >= min_range
    d_unit = direction / (np.linalg.norm(direction) + 1e-9)
    cos_vals = np.dot(v, d_unit) / dist
    mask &= cos_vals >= cos_fov
    return np.nonzero(mask)[0]


def resolve_visible_feature_path(quiver_path):
    base_dir = os.path.dirname(quiver_path)
    base_name = os.path.basename(quiver_path)
    candidates = [
        os.path.join(base_dir, "visible_features_per_iteration.txt"),
        append_suffix_before_extension(quiver_path, "_visible_idx"),
        os.path.join(base_dir, "quivers_path_yaw_visible_idx.txt"),
        os.path.join(base_dir, "initial_quivers_path_yaw_visible_idx.txt"),
        os.path.join(base_dir, "quivers_visible_idx.txt"),
        os.path.join(base_dir, "initial_quivers_visible_idx.txt"),
    ]
    if base_name == "quivers_path_yaw.txt":
        candidates.append(os.path.join(base_dir, "initial_quivers_path_yaw_visible_idx.txt"))
    if base_name == "quivers.txt":
        candidates.append(os.path.join(base_dir, "initial_quivers_visible_idx.txt"))
    for cand in candidates:
        if os.path.isfile(cand):
            return cand
    return ""


def load_visible_feature_indices(path, pose_index, expected_iters):
    out = []
    current = None
    in_block = False
    with open(path, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                if in_block:
                    if current is None:
                        out.append(np.empty((0,), dtype=int))
                    else:
                        out.append(np.asarray(current, dtype=int))
                    current = None
                    in_block = False
                continue
            if line.startswith("#"):
                continue
            in_block = True
            parts = [p for p in line.replace(",", " ").split() if p]
            if not parts:
                continue
            try:
                cur_pose = int(parts[0])
            except ValueError:
                continue
            if cur_pose != pose_index:
                continue
            vals = []
            for token in parts[1:]:
                try:
                    vals.append(int(token))
                except ValueError:
                    continue
            current = vals
    if in_block:
        if current is None:
            out.append(np.empty((0,), dtype=int))
        else:
            out.append(np.asarray(current, dtype=int))

    if expected_iters >= 0:
        if len(out) < expected_iters:
            for _ in range(expected_iters - len(out)):
                out.append(np.empty((0,), dtype=int))
        elif len(out) > expected_iters:
            out = out[:expected_iters]
    return out


def build_markers(frame, namespace, pos_all, dir_all, pose_index, points,
                  cos_fov, min_range, max_range, path_width, arrow_len,
                  arrow_shaft_d, arrow_head_d, arrow_head_len, point_scale,
                  show_path_arrows, visible_idx_per_iter=None):
    n_iter = pos_all.shape[0]
    final_path = pos_all[-1]

    msg = MarkerArray()
    delete_all = marker_base(frame, namespace, 999999, Marker.CUBE)
    delete_all.action = Marker.DELETEALL
    msg.markers.append(delete_all)

    path = marker_base(frame, "{}_path".format(namespace), 0, Marker.LINE_STRIP)
    path.scale.x = path_width
    path.color = make_color(0.05, 0.45, 0.95, 0.75)
    path.points = [np_to_point(p) for p in final_path]
    msg.markers.append(path)

    if show_path_arrows and final_path.shape[0] > 1:
        for i in range(final_path.shape[0] - 1):
            seg = marker_base(frame, "{}_path_arrows".format(namespace), 100 + i, Marker.ARROW)
            seg.scale.x = arrow_shaft_d * 0.8
            seg.scale.y = arrow_head_d * 0.7
            seg.scale.z = arrow_head_len * 0.7
            seg.color = make_color(0.05, 0.45, 0.95, 0.85)
            seg.points = [np_to_point(final_path[i]), np_to_point(final_path[i + 1])]
            msg.markers.append(seg)

    pose_trail = marker_base(frame, "{}_pose_trail".format(namespace), 1, Marker.SPHERE_LIST)
    pose_trail.scale.x = point_scale * 1.25
    pose_trail.scale.y = point_scale * 1.25
    pose_trail.scale.z = point_scale * 1.25

    init_count = 0
    final_count = 0

    for it in range(n_iter):
        t = 0.0 if n_iter <= 1 else float(it) / float(n_iter - 1)
        color_arrow = interp_red_green(t, 0.95)
        color_pts = interp_red_green(t, 0.22 if it < n_iter - 1 else 0.30)

        pos = pos_all[it, pose_index]
        direction = dir_all[it, pose_index]
        d_unit = direction / (np.linalg.norm(direction) + 1e-9)
        end = pos + d_unit * arrow_len

        pose_trail.points.append(np_to_point(pos))
        pose_trail.colors.append(interp_red_green(t, 0.85))

        arrow = marker_base(frame, "{}_arrows".format(namespace), 1000 + it, Marker.ARROW)
        arrow.scale.x = arrow_shaft_d
        arrow.scale.y = arrow_head_d
        arrow.scale.z = arrow_head_len
        arrow.color = color_arrow
        arrow.points = [np_to_point(pos), np_to_point(end)]
        msg.markers.append(arrow)

        if visible_idx_per_iter is not None and it < len(visible_idx_per_iter):
            vis_idx = visible_idx_per_iter[it]
            if vis_idx.size > 0:
                vis_idx = vis_idx[(vis_idx >= 0) & (vis_idx < points.shape[0])]
        else:
            if cos_fov is None:
                raise RuntimeError(
                    "No saved visible-feature sidecar found and --fov-deg was not provided.")
            vis_idx = compute_visible_indices(pos, direction, points, cos_fov, min_range, max_range)
        if it == 0:
            init_count = int(vis_idx.size)
        if it == n_iter - 1:
            final_count = int(vis_idx.size)

        if vis_idx.size > 0:
            pts = marker_base(frame, "{}_features".format(namespace), 2000 + it, Marker.POINTS)
            pts.scale.x = point_scale
            pts.scale.y = point_scale
            pts.color = color_pts
            pts.points = [np_to_point(points[j]) for j in vis_idx]
            msg.markers.append(pts)

    msg.markers.append(pose_trail)

    init_text = marker_base(frame, "{}_labels".format(namespace), 3000, Marker.TEXT_VIEW_FACING)
    init_text.scale.z = point_scale * 12.0
    init_text.color = make_color(1.0, 0.3, 0.3, 0.95)
    init_text.pose.position = np_to_point(pos_all[0, pose_index])
    init_text.pose.position.z -= point_scale * 8.0
    init_text.text = "init: {}".format(init_count)
    msg.markers.append(init_text)

    final_text = marker_base(frame, "{}_labels".format(namespace), 3001, Marker.TEXT_VIEW_FACING)
    final_text.scale.z = point_scale * 12.0
    final_text.color = make_color(0.2, 0.95, 0.25, 0.95)
    final_text.pose.position = np_to_point(pos_all[-1, pose_index])
    final_text.pose.position.z -= point_scale * 12.0
    final_text.text = "final: {}".format(final_count)
    msg.markers.append(final_text)

    return msg


def parse_args():
    parser = argparse.ArgumentParser(
        description="Static FoV optimization story viewer for one pose."
    )
    parser.add_argument("--quivers", required=True, help="Path to per_iteration_quivers.txt or legacy quiver file.")
    parser.add_argument("--points", required=True, help="Path to points3D.txt or xyz file.")
    parser.add_argument("--points-cols", default="0,1,2")
    parser.add_argument("--frame", default="world")
    parser.add_argument("--topic", default="fov_pose_story/markers")
    parser.add_argument("--namespace", default="fov_pose_story")
    parser.add_argument("--pose-index", type=int, default=7)
    parser.add_argument("--fov-deg", type=float, default=None)
    parser.add_argument("--min-range", type=float, default=0.0)
    parser.add_argument("--max-range", type=float, default=-1.0)
    parser.add_argument("--point-stride", type=int, default=1)
    parser.add_argument("--max-points", type=int, default=8000)
    parser.add_argument("--path-width-scale", type=float, default=0.006)
    parser.add_argument("--arrow-scale", type=float, default=0.06)
    parser.add_argument("--feature-scale", type=float, default=0.015)
    parser.add_argument("--require-saved-features", action="store_true")
    parser.add_argument("--show-path-arrows", action="store_true")
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rospy.init_node("fov_pose_story_viz", anonymous=True)

    quiver_path = os.path.abspath(os.path.expanduser(args.quivers))
    points_path = os.path.abspath(os.path.expanduser(args.points))
    raw = parse_quiver_blocks(quiver_path, min_cols=6)
    pos_all = raw[:, :, :3]
    dir_all = raw[:, :, 3:6]

    if args.pose_index < 0 or args.pose_index >= pos_all.shape[1]:
        raise ValueError("--pose-index {} out of range [0, {}]".format(
            args.pose_index, pos_all.shape[1] - 1))

    visible_feature_path = resolve_visible_feature_path(quiver_path)
    visible_idx_per_iter = None
    if args.require_saved_features and not visible_feature_path:
        raise RuntimeError(
            "Saved visible-feature sidecar is required for {} but none was found.".format(
                quiver_path))
    point_stride = max(1, args.point_stride)
    max_points = max(0, args.max_points)
    if visible_feature_path:
        # Sidecar indices refer to the full point file.
        point_stride = 1
        max_points = 0
    points = load_points(
        points_path,
        parse_cols(args.points_cols),
        stride=point_stride,
        max_points=max_points,
    )
    if visible_feature_path:
        visible_idx_per_iter = load_visible_feature_indices(
            visible_feature_path, args.pose_index, pos_all.shape[0])

    all_positions = pos_all.reshape(-1, 3)
    extent = all_positions.max(axis=0) - all_positions.min(axis=0)
    diag = float(np.linalg.norm(extent))
    if diag < 1e-6:
        diag = 1.0

    path_width = max(0.02, diag * max(0.001, args.path_width_scale))
    arrow_len = max(path_width * 5.0, diag * max(0.005, args.arrow_scale))
    arrow_shaft_d = path_width * 1.1
    arrow_head_d = path_width * 2.2
    arrow_head_len = path_width * 2.8
    point_scale = max(path_width * 0.6, diag * max(0.001, args.feature_scale))
    cos_fov = None
    if not visible_feature_path:
        if args.fov_deg is None:
            raise ValueError(
                "--fov-deg is required when no saved visible-feature sidecar is available.")
        cos_fov = np.cos(np.deg2rad(args.fov_deg) * 0.5)

    msg = build_markers(
        frame=args.frame,
        namespace=args.namespace,
        pos_all=pos_all,
        dir_all=dir_all,
        pose_index=args.pose_index,
        points=points,
        cos_fov=cos_fov,
        min_range=args.min_range,
        max_range=args.max_range,
        path_width=path_width,
        arrow_len=arrow_len,
        arrow_shaft_d=arrow_shaft_d,
        arrow_head_d=arrow_head_d,
        arrow_head_len=arrow_head_len,
        point_scale=point_scale,
        show_path_arrows=args.show_path_arrows,
        visible_idx_per_iter=visible_idx_per_iter,
    )

    pub = rospy.Publisher(args.topic, MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.3)
    pub.publish(msg)

    rospy.loginfo("FoV pose story visualization")
    rospy.loginfo("- quivers: %s", quiver_path)
    rospy.loginfo("- points: %s", points_path)
    rospy.loginfo("- pose index: %d", args.pose_index)
    rospy.loginfo("- iterations: %d", pos_all.shape[0])
    rospy.loginfo("- topic: %s", args.topic)
    if visible_feature_path:
        rospy.loginfo("- visible features sidecar: %s", visible_feature_path)
    else:
        rospy.loginfo("- visible features sidecar: not found, using cone-only recompute")

    rospy.spin()


if __name__ == "__main__":
    main()
