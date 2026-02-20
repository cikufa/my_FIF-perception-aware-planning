#!/usr/bin/env python3
"""RViz debug viewer for FoV optimization.

This node animates per-iteration camera quivers and overlays per-pose
visibility / feature-count diagnostics computed from a point set.

Inputs:
  - quivers file: blocks separated by blank lines, each line is x,y,z,dx,dy,dz
  - points file: CSV/space separated XYZ (optionally COLMAP points3D.txt)

Outputs (MarkerArray topic):
  - current iteration path (LINE_STRIP)
  - view arrows (LINE_LIST)
  - pose spheres colored by count/score (SPHERE_LIST)
  - optional visibility links for a selected pose (LINE_LIST)
  - optional visible points for a selected pose (POINTS)
  - status text (TEXT_VIEW_FACING)
"""

import argparse
from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def make_color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def np_to_point(xyz: np.ndarray) -> Point:
    msg = Point()
    msg.x = float(xyz[0])
    msg.y = float(xyz[1])
    msg.z = float(xyz[2])
    return msg


def marker_base(frame: str, ns: str, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def parse_quiver_blocks(path: Path) -> np.ndarray:
    blocks: List[np.ndarray] = []
    cur: List[List[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                if cur:
                    blocks.append(np.asarray(cur, dtype=float))
                    cur = []
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) >= 6:
                cur.append([float(v) for v in parts[:6]])
    if cur:
        blocks.append(np.asarray(cur, dtype=float))

    if not blocks:
        raise RuntimeError(f"No iteration blocks found in {path}")

    min_n_pose = min(len(b) for b in blocks)
    blocks = [b[:min_n_pose] for b in blocks]
    return np.stack(blocks, axis=0)


def parse_cols(cols: str) -> Tuple[int, int, int]:
    parts = [p.strip() for p in cols.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError("--points-cols must be 3 comma-separated integers")
    return int(parts[0]), int(parts[1]), int(parts[2])


def load_points(path: Path, cols: Tuple[int, int, int], stride: int, max_points: int) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Points file does not exist: {path}")
    pts: List[List[float]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
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
        rng = np.random.default_rng(7)
        idx = rng.choice(arr.shape[0], size=max_points, replace=False)
        arr = arr[idx]
    return arr


def color_from_value(val: float, vmin: float, vmax: float) -> ColorRGBA:
    if vmax <= vmin:
        t = 0.0
    else:
        t = (val - vmin) / (vmax - vmin)
        t = max(0.0, min(1.0, t))
    # blue -> red gradient
    r = 0.15 + 0.85 * t
    g = 0.35 * (1.0 - t)
    b = 1.0 - 0.75 * t
    return make_color(r, g, b, 0.9)


def compute_visibility(
    pos_it: np.ndarray,
    dir_it: np.ndarray,
    points: np.ndarray,
    cos_fov: float,
    min_range: float,
    max_range: float,
    use_sigmoid: bool,
    sigmoid_k: float,
) -> Tuple[np.ndarray, List[np.ndarray]]:
    counts = np.zeros((pos_it.shape[0],), dtype=float)
    vis_indices: List[np.ndarray] = []
    if points.size == 0:
        for _ in range(pos_it.shape[0]):
            vis_indices.append(np.empty((0,), dtype=int))
        return counts, vis_indices

    for i, (p, d) in enumerate(zip(pos_it, dir_it)):
        v = points - p[None, :]
        dist = np.linalg.norm(v, axis=1) + 1e-9
        if max_range > 0:
            mask = dist <= max_range
        else:
            mask = np.ones_like(dist, dtype=bool)
        if min_range > 0:
            mask &= dist >= min_range
        d_unit = d / (np.linalg.norm(d) + 1e-9)
        cos_vals = (v @ d_unit) / dist
        mask &= cos_vals >= cos_fov
        idx = np.nonzero(mask)[0]
        vis_indices.append(idx)
        if use_sigmoid:
            # soft visibility score
            w = 1.0 / (1.0 + np.exp(-sigmoid_k * (cos_vals - cos_fov)))
            counts[i] = float(w[mask].sum())
        else:
            counts[i] = float(idx.size)
    return counts, vis_indices


def build_markers(
    frame: str,
    ns: str,
    pos_it: np.ndarray,
    dir_it: np.ndarray,
    counts: np.ndarray,
    vis_idx: List[np.ndarray],
    points: np.ndarray,
    show_links: bool,
    show_visible_points: bool,
    pose_index: int,
    arrow_len: float,
    path_width: float,
    pose_scale: float,
    text_height: float,
    text_every: int,
    links_every: int,
) -> List[Marker]:
    markers: List[Marker] = []

    # Path
    path = marker_base(frame, f"{ns}_path", 0, Marker.LINE_STRIP)
    path.scale.x = path_width
    path.color = make_color(0.05, 0.55, 1.0, 0.85)
    path.points = [np_to_point(xyz) for xyz in pos_it]
    markers.append(path)

    # View arrows
    view = marker_base(frame, f"{ns}_view", 1, Marker.LINE_LIST)
    view.scale.x = path_width * 0.7
    view.color = make_color(1.0, 0.35, 0.2, 0.9)
    for xyz, d in zip(pos_it, dir_it):
        dn = d / (np.linalg.norm(d) + 1e-9)
        view.points.extend([np_to_point(xyz), np_to_point(xyz + dn * arrow_len)])
        view.colors.extend([view.color, view.color])
    markers.append(view)

    # Pose spheres colored by count
    pose_marker = marker_base(frame, f"{ns}_pose", 2, Marker.SPHERE_LIST)
    pose_marker.scale.x = pose_scale
    pose_marker.scale.y = pose_scale
    pose_marker.scale.z = pose_scale
    if counts.size > 0:
        vmin = float(np.min(counts))
        vmax = float(np.max(counts))
    else:
        vmin = 0.0
        vmax = 1.0
    for i, xyz in enumerate(pos_it):
        pose_marker.points.append(np_to_point(xyz))
        pose_marker.colors.append(color_from_value(float(counts[i]), vmin, vmax))
    markers.append(pose_marker)

    # Optional text labels
    if text_every > 0:
        text_marker = marker_base(frame, f"{ns}_text", 3, Marker.TEXT_VIEW_FACING)
        text_marker.scale.z = text_height
        text_marker.color = make_color(1.0, 1.0, 1.0, 0.9)
        text_marker.points = []
        text_marker.text = ""
        # RViz TEXT_VIEW_FACING ignores points; publish multiple markers instead
        text_markers: List[Marker] = []
        for i, xyz in enumerate(pos_it):
            if (i % text_every) != 0:
                continue
            tm = marker_base(frame, f"{ns}_text", 1000 + i, Marker.TEXT_VIEW_FACING)
            tm.scale.z = text_height
            tm.color = make_color(1.0, 1.0, 1.0, 0.9)
            tm.pose.position = np_to_point(xyz)
            tm.pose.position.z += text_height * 1.2
            tm.text = f"{int(round(counts[i]))}"
            text_markers.append(tm)
        markers.extend(text_markers)

    # Optional links/visible points for a selected pose
    if pose_index >= 0 and pose_index < pos_it.shape[0] and points.size > 0:
        sel_idx = vis_idx[pose_index]
        if links_every > 1 and sel_idx.size > 0:
            sel_idx = sel_idx[::links_every]

        if show_links and sel_idx.size > 0:
            links = marker_base(frame, f"{ns}_links", 4, Marker.LINE_LIST)
            links.scale.x = path_width * 0.5
            links.color = make_color(1.0, 0.8, 0.1, 0.55)
            p0 = pos_it[pose_index]
            for pi in sel_idx:
                links.points.extend([np_to_point(p0), np_to_point(points[pi])])
                links.colors.extend([links.color, links.color])
            markers.append(links)

        if show_visible_points and sel_idx.size > 0:
            vis_pts = marker_base(frame, f"{ns}_vis_pts", 5, Marker.POINTS)
            vis_pts.scale.x = pose_scale * 0.6
            vis_pts.scale.y = pose_scale * 0.6
            vis_pts.color = make_color(0.95, 0.95, 0.2, 0.6)
            for pi in sel_idx:
                vis_pts.points.append(np_to_point(points[pi]))
            markers.append(vis_pts)

    return markers


def publish_delete_all(pub: rospy.Publisher, frame: str, ns: str) -> None:
    marker = marker_base(frame, ns, 9999, Marker.CUBE)
    marker.action = Marker.DELETEALL
    msg = MarkerArray()
    msg.markers.append(marker)
    pub.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Animate FoV optimization quivers with visibility/feature debug overlays."
    )
    parser.add_argument("--quivers", required=True, help="Path to quivers file.")
    parser.add_argument("--points", default="", help="Optional points file (XYZ or COLMAP points3D.txt).")
    parser.add_argument("--points-cols", default="0,1,2",
                        help="Columns for XYZ (0-based), e.g. '1,2,3' for COLMAP points3D.txt.")
    parser.add_argument("--frame", default="world", help="RViz fixed frame.")
    parser.add_argument("--topic", default="fov_debug/markers", help="MarkerArray topic.")
    parser.add_argument("--namespace", default="fov_debug", help="Marker namespace prefix.")
    parser.add_argument("--rate", type=float, default=2.0, help="Playback rate in Hz.")
    parser.add_argument("--start-iter", type=int, default=0, help="First iteration to display.")
    parser.add_argument("--end-iter", type=int, default=-1, help="Last iteration to display (-1 = final).")
    parser.add_argument("--no-loop", action="store_true", help="Play once and stop.")

    parser.add_argument("--fov-deg", type=float, default=90.0, help="Horizontal FOV in degrees.")
    parser.add_argument("--min-range", type=float, default=0.0, help="Min visibility range (m).")
    parser.add_argument("--max-range", type=float, default=-1.0, help="Max visibility range (m). -1 disables.")
    parser.add_argument("--use-sigmoid", action="store_true",
                        help="Use sigmoid visibility score instead of hard count.")
    parser.add_argument("--sigmoid-k", type=float, default=15.0, help="Sigmoid sharpness.")

    parser.add_argument("--pose-index", type=int, default=-1,
                        help="Show visibility links/points for this pose index only.")
    parser.add_argument("--show-links", action="store_true", help="Show links from pose to visible points.")
    parser.add_argument("--show-visible-points", action="store_true", help="Show visible points for pose.")
    parser.add_argument("--links-every", type=int, default=5,
                        help="Downsample visible links (every Nth).")

    parser.add_argument("--point-stride", type=int, default=4, help="Stride for points subsampling.")
    parser.add_argument("--max-points", type=int, default=8000, help="Max points loaded.")

    parser.add_argument("--arrow-scale", type=float, default=0.05,
                        help="Arrow length as ratio of path diagonal.")
    parser.add_argument("--path-width-scale", type=float, default=0.006,
                        help="Path width as ratio of path diagonal.")
    parser.add_argument("--pose-scale", type=float, default=0.03,
                        help="Pose sphere diameter as ratio of path diagonal.")
    parser.add_argument("--text-every", type=int, default=5,
                        help="Show count text every N poses (0 disables).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rospy.init_node("fov_debug_viz", anonymous=True)

    quiver_path = Path(args.quivers).expanduser().resolve()
    if not quiver_path.exists():
        raise FileNotFoundError(f"Quiver file does not exist: {quiver_path}")
    raw = parse_quiver_blocks(quiver_path)
    pos = raw[:, :, :3]
    dirs = raw[:, :, 3:6]
    n_iter, n_pose, _ = pos.shape

    points = np.empty((0, 3), dtype=float)
    if args.points:
        points = load_points(
            Path(args.points).expanduser().resolve(),
            parse_cols(args.points_cols),
            stride=max(1, args.point_stride),
            max_points=max(0, args.max_points),
        )

    start_iter = max(0, args.start_iter)
    end_iter = n_iter - 1 if args.end_iter < 0 else min(n_iter - 1, args.end_iter)
    if start_iter > end_iter:
        raise ValueError("start-iter must be <= end-iter")
    iter_ids = list(range(start_iter, end_iter + 1))

    all_positions = pos.reshape(-1, 3)
    extent = all_positions.max(axis=0) - all_positions.min(axis=0)
    diag = float(np.linalg.norm(extent))
    if diag < 1e-6:
        diag = 1.0

    path_width = max(0.02, diag * max(0.001, args.path_width_scale))
    arrow_len = max(path_width * 3.0, diag * max(0.005, args.arrow_scale))
    pose_scale = max(path_width * 1.5, diag * max(0.002, args.pose_scale))
    text_height = max(path_width * 3.5, diag * 0.03)

    cos_fov = np.cos(np.deg2rad(args.fov_deg) * 0.5)

    pub = rospy.Publisher(args.topic, MarkerArray, queue_size=1)
    rospy.sleep(0.2)
    publish_delete_all(pub, args.frame, args.namespace)
    rospy.sleep(0.1)

    rospy.loginfo("FoV debug visualization")
    rospy.loginfo("- quivers: %s", quiver_path)
    rospy.loginfo("- iterations: %d (showing %d..%d)", n_iter, start_iter, end_iter)
    rospy.loginfo("- poses per iteration: %d", n_pose)
    rospy.loginfo("- points loaded: %d", points.shape[0])
    rospy.loginfo("- topic: %s", args.topic)

    rate_hz = max(0.1, float(args.rate))
    rate = rospy.Rate(rate_hz)

    idx = 0
    while not rospy.is_shutdown():
        iter_id = iter_ids[idx]
        pos_it = pos[iter_id]
        dir_it = dirs[iter_id]
        counts, vis_idx = compute_visibility(
            pos_it,
            dir_it,
            points,
            cos_fov,
            args.min_range,
            args.max_range,
            args.use_sigmoid,
            args.sigmoid_k,
        )

        markers = build_markers(
            frame=args.frame,
            ns=args.namespace,
            pos_it=pos_it,
            dir_it=dir_it,
            counts=counts,
            vis_idx=vis_idx,
            points=points,
            show_links=args.show_links,
            show_visible_points=args.show_visible_points,
            pose_index=args.pose_index,
            arrow_len=arrow_len,
            path_width=path_width,
            pose_scale=pose_scale,
            text_height=text_height,
            text_every=args.text_every,
            links_every=max(1, args.links_every),
        )

        # Status text
        status = marker_base(args.frame, f"{args.namespace}_status", 9000, Marker.TEXT_VIEW_FACING)
        status.scale.z = text_height * 1.1
        status.color = make_color(0.95, 0.95, 0.95, 0.9)
        status.pose.position = np_to_point(pos_it[0])
        status.pose.position.z += text_height * 2.2
        status.text = (
            f"iter {iter_id}/{n_iter - 1} | "
            f"count min/mean/max: {counts.min():.1f}/{counts.mean():.1f}/{counts.max():.1f}"
        )
        markers.append(status)

        msg = MarkerArray()
        msg.markers.extend(markers)
        pub.publish(msg)

        if idx == len(iter_ids) - 1:
            if args.no_loop:
                rospy.loginfo("Reached final displayed iteration %d. Holding markers.", iter_id)
                break
            idx = 0
        else:
            idx += 1
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()
