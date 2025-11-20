#!/usr/bin/env python3
"""Visualize warehouse camera paths described by UE-styled stamped poses.

The script publishes two sets of markers (paths + view directions) that RViz can show
alongside your warehouse environment. Run RViz with your warehouse config, then launch
this node to see the before/after exposures colored differently.
"""

import argparse
import math
import os
from typing import List, Sequence, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def parse_rgb(rgb_str: str) -> Tuple[float, float, float]:
    comps = [comp.strip() for comp in rgb_str.split(",")]
    if len(comps) != 3:
        raise ValueError(f"Expected three comma-separated values, got: {rgb_str}")
    rgb = []
    for comp in comps:
        value = float(comp)
        rgb.append(max(0.0, min(1.0, value)))
    return tuple(rgb)


def color_msg(rgb: Sequence[float], alpha: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = float(rgb[0])
    msg.g = float(rgb[1])
    msg.b = float(rgb[2])
    msg.a = alpha
    return msg


def rot_x(deg: float) -> np.ndarray:
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    mat = np.eye(3)
    mat[1, 1] = c
    mat[1, 2] = -s
    mat[2, 1] = s
    mat[2, 2] = c
    return mat


def rot_y(deg: float) -> np.ndarray:
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    mat = np.eye(3)
    mat[0, 0] = c
    mat[0, 2] = s
    mat[2, 0] = -s
    mat[2, 2] = c
    return mat


def rot_z(deg: float) -> np.ndarray:
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    mat = np.eye(3)
    mat[0, 0] = c
    mat[0, 1] = -s
    mat[1, 0] = s
    mat[1, 1] = c
    return mat


def euler_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Imitate Unreal's conversion used in the existing C++ helper."""
    roll_mat = rot_x(-roll)
    pitch_mat = rot_y(-pitch)
    yaw_mat = rot_z(yaw)
    return yaw_mat @ pitch_mat @ roll_mat


def get_t_w_wue() -> np.ndarray:
    mat = np.eye(4)
    mat[1, 1] = -1.0
    return mat


def get_t_c_cue() -> np.ndarray:
    mat = np.zeros((4, 4))
    mat[0, 1] = 1.0
    mat[1, 2] = -1.0
    mat[2, 0] = 1.0
    mat[3, 3] = 1.0
    return mat


def ue_to_standard_matrix(x: float, y: float, z: float,
                          pitch: float, yaw: float, roll: float) -> np.ndarray:
    """Return Twc in standard frame (same as exp_utils::loadStampedUePoses)."""
    Twc_ue = np.eye(4)
    Twc_ue[:3, :3] = euler_to_rotmat(roll, pitch, yaw)
    Twc_ue[:3, 3] = np.array([x, y, z])

    return get_t_w_wue() @ Twc_ue @ np.linalg.inv(get_t_c_cue())


def load_poses(fn: str) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Parse UE poses and return pairs of (position, view_direction)."""
    poses = []
    if not os.path.isfile(fn):
        raise FileNotFoundError(f"{fn} does not exist")
    with open(fn, "r") as fp:
        for line in fp:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 7:
                continue
            _, x, y, z, pitch, yaw, roll = parts[:7]
            Twc = ue_to_standard_matrix(float(x), float(y), float(z),
                                        float(pitch), float(yaw), float(roll))
            position = Twc[:3, 3]
            rotation = Twc[:3, :3]
            view_dir = -rotation[:, 2]
            view_dir /= np.linalg.norm(view_dir) + 1e-9
            poses.append((position, view_dir))
    return poses


def make_marker(base_id: int,
                ns: str,
                frame: str,
                marker_type: int,
                color: ColorRGBA,
                scale: float) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = base_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.color = color
    return marker


def create_path_marker(poses: List[Tuple[np.ndarray, np.ndarray]],
                       frame: str,
                       color: ColorRGBA,
                       marker_id: int,
                       ns: str) -> Marker:
    marker = make_marker(marker_id, ns, frame, Marker.LINE_STRIP, color, 0.05)
    marker.scale.x = 0.04
    for position, _ in poses:
        point = Point()
        point.x, point.y, point.z = position.tolist()
        marker.points.append(point)
    return marker


def create_view_marker(poses: List[Tuple[np.ndarray, np.ndarray]],
                       frame: str,
                       color: ColorRGBA,
                       marker_id: int,
                       ns: str,
                       arrow_length: float) -> Marker:
    marker = make_marker(marker_id, ns, frame, Marker.LINE_LIST, color, 0.02)
    for position, view_dir in poses:
        start = Point()
        end = Point()
        start.x, start.y, start.z = position.tolist()
        target = position + view_dir * arrow_length
        end.x, end.y, end.z = target.tolist()
        marker.points.extend([start, end])
        marker.colors.extend([color, color])
    return marker


def compute_arrow_length(poses: Sequence[Tuple[np.ndarray, np.ndarray]]) -> float:
    if not poses:
        return 0.5
    positions = np.vstack([pose[0] for pose in poses])
    span = positions.max(axis=0) - positions.min(axis=0)
    diag = np.linalg.norm(span)
    return max(0.5, diag * 0.2)


def publish_datasets(datasets: Sequence[Tuple[str, List[Tuple[np.ndarray, np.ndarray]],
                                               ColorRGBA]],
                     frame: str) -> None:
    """Publish all datasets in a single MarkerArray."""
    pub = rospy.Publisher("stamped_pose_viz", MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.2)
    marker_array = MarkerArray()
    base_id = 0
    for idx, (label, poses, color) in enumerate(datasets):
        if not poses:
            rospy.logwarn("No poses in %s, skipping visualization.", label)
            continue
        length = compute_arrow_length(poses)
        ns_path = f"{label}_path"
        path_marker = create_path_marker(poses, frame, color, base_id, ns_path)
        marker_array.markers.append(path_marker)
        base_id += 1
        ns_view = f"{label}_view"
        view_marker = create_view_marker(poses, frame, color, base_id, ns_view, length)
        marker_array.markers.append(view_marker)
        base_id += 1
    pub.publish(marker_array)
    rospy.loginfo("Published %d marker sets for frame %s", len(datasets), frame)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize before/after UE stamped poses in RViz.")
    parser.add_argument("--before", help="Path to the before optimization stamped_Twc_ue.txt file")
    parser.add_argument("--after", help="Path to the after optimization stamped_Twc_ue.txt file")
    parser.add_argument("--before-color", default="0.0,0.6,1.0",
                        help="RGB color for the before path (comma separated values in [0,1]).")
    parser.add_argument("--after-color", default="1.0,0.4,0.0",
                        help="RGB color for the after path (comma separated values in [0,1]).")
    parser.add_argument("--frame", default="world",
                        help="TF frame id used by RViz.")
    args = parser.parse_args()

    if not args.before and not args.after:
        parser.error("At least one of --before or --after must be specified.")

    rospy.init_node("stamped_pose_visualizer", anonymous=True)

    datasets = []
    if args.before:
        poses_before = load_poses(args.before)
        datasets.append(("before", poses_before, color_msg(parse_rgb(args.before_color))))
    if args.after:
        poses_after = load_poses(args.after)
        datasets.append(("after", poses_after, color_msg(parse_rgb(args.after_color))))

    publish_datasets(datasets, args.frame)
    rospy.loginfo("Visualization ready; open RViz with your warehouse environment.")
    rospy.spin()


if __name__ == "__main__":
    main()
