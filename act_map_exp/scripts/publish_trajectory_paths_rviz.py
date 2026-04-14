#!/usr/bin/env python3
"""
Publish baseline vs optimized trajectories as nav_msgs/Path for RViz comparison.

Reads stamped SE(3) files (same format as analyze_pose_errors: time + row-major 4x4).
Translation is taken from the last column of the upper 3x4 block (camera/world positions).

Typical layout:
  <root>/<traj>/<traj>_none/stamped_Twc_path_yaw.txt          (baseline)
  <root>/<traj>/<traj>_none/optimized_path_yaw/stamped_Twc_path_yaw.txt  (ours)

Example (after sourcing your ROS workspace):
  python3 publish_trajectory_paths_rviz.py \\
    --root /path/to/trace_r1_a30 --traj mid --rate 10

In RViz: Add Path display, subscribe to /baseline_path and /optimized_path (set line width,
distinct colors). Set Fixed Frame to match --frame-id (default: map).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def load_xyz_from_stamped(path: Path) -> np.ndarray:
    data = np.loadtxt(path, comments="#")
    if data.size == 0:
        return np.zeros((0, 3))
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 17:
        raise ValueError(f"Expected >=17 columns (time + 4x4), got {data.shape[1]} in {path}")
    mat = data[:, 1:].reshape(-1, 4, 4)
    return mat[:, :3, 3]


def default_paths(root: Path, traj: str) -> tuple[Path, Path]:
    base = root / traj / f"{traj}_none"
    baseline = base / "stamped_Twc_path_yaw.txt"
    if not baseline.exists():
        alt = base / "stamped_Twc.txt"
        if alt.exists():
            baseline = alt
    optimized = base / "optimized_path_yaw" / "stamped_Twc_path_yaw.txt"
    if not optimized.exists():
        alt2 = base / "optimized_path_yaw" / "stamped_Twc.txt"
        if alt2.exists():
            optimized = alt2
    return baseline, optimized


def main() -> None:
    try:
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        from std_msgs.msg import Header
    except ImportError as import_err:
        print(
            "Requires ROS Python: rospy, nav_msgs, geometry_msgs, std_msgs\n"
            "Source your catkin workspace, then retry.",
            file=sys.stderr,
        )
        raise SystemExit(1) from import_err

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="Experiment root (e.g. trace_r1_a30)")
    parser.add_argument("--traj", type=str, required=True, help="Trajectory name (e.g. mid, bigU)")
    parser.add_argument(
        "--baseline",
        type=Path,
        default=None,
        help="Override baseline stamped pose file",
    )
    parser.add_argument(
        "--optimized",
        type=Path,
        default=None,
        help="Override optimized stamped pose file",
    )
    parser.add_argument("--frame-id", type=str, default="map", help="TF frame for Path header")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate (Hz), 0 = latch once")
    parser.add_argument(
        "--topics",
        nargs=2,
        default=["/baseline_path", "/optimized_path"],
        metavar=("BASELINE", "OPTIMIZED"),
        help="ROS topic names for the two paths",
    )
    args = parser.parse_args()

    if args.baseline and args.optimized:
        baseline_p, opt_p = args.baseline, args.optimized
    else:
        baseline_p, opt_p = default_paths(args.root, args.traj)

    if not baseline_p.is_file():
        print(f"Missing baseline file: {baseline_p}", file=sys.stderr)
        raise SystemExit(2)
    if not opt_p.is_file():
        print(f"Missing optimized file: {opt_p}", file=sys.stderr)
        raise SystemExit(2)

    xyz_b = load_xyz_from_stamped(baseline_p)
    xyz_o = load_xyz_from_stamped(opt_p)
    if xyz_b.shape[0] == 0 or xyz_o.shape[0] == 0:
        print("One trajectory has no poses.", file=sys.stderr)
        raise SystemExit(3)

    rospy.init_node("trajectory_paths_compare", anonymous=True)
    pub_b = rospy.Publisher(args.topics[0], Path, queue_size=1, latch=True)
    pub_o = rospy.Publisher(args.topics[1], Path, queue_size=1, latch=True)

    def build_path(xyz: np.ndarray) -> Path:
        p = Path()
        p.header = Header()
        p.header.frame_id = args.frame_id
        p.poses = []
        for i in range(xyz.shape[0]):
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = float(xyz[i, 0])
            ps.pose.position.y = float(xyz[i, 1])
            ps.pose.position.z = float(xyz[i, 2])
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p

    path_b = build_path(xyz_b)
    path_o = build_path(xyz_o)

    rospy.loginfo("Baseline: %s (%d poses)", baseline_p, len(path_b.poses))
    rospy.loginfo("Optimized: %s (%d poses)", opt_p, len(path_o.poses))
    rospy.loginfo("Publishing to %s and %s (frame=%s)", args.topics[0], args.topics[1], args.frame_id)

    rate = rospy.Rate(args.rate) if args.rate > 0 else None
    while not rospy.is_shutdown():
        path_b.header.stamp = rospy.Time.now()
        path_o.header.stamp = rospy.Time.now()
        for ps in path_b.poses:
            ps.header.stamp = path_b.header.stamp
        for ps in path_o.poses:
            ps.header.stamp = path_o.header.stamp
        pub_b.publish(path_b)
        pub_o.publish(path_o)
        if rate is None:
            rospy.spin()
            break
        rate.sleep()


if __name__ == "__main__":
    main()
