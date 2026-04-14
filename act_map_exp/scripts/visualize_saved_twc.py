#!/usr/bin/env python3
"""
Convenience script to visualize saved stamped_Twc paths through the existing
quad_rrt_node service (no new visualization node needed).
"""

import argparse
import sys
import rospy
from act_map_exp.srv import VisualizeStampedPoses, VisualizeStampedPosesRequest


def parse_rgb(rgb_str: str):
    parts = [float(v) for v in rgb_str.split(",")]
    if len(parts) != 3:
        raise ValueError("RGB must have exactly 3 comma-separated values")
    return parts


def main():
    parser = argparse.ArgumentParser(
        description="Publish saved stamped_Twc paths via quad_rrt visualize_saved_rrt service.")
    parser.add_argument("--file-a", required=True,
                        help="Path to first stamped_Twc.txt")
    parser.add_argument("--file-b", default="",
                        help="Optional second stamped_Twc.txt")
    parser.add_argument("--color-a", default="0.0,1.0,0.0",
                        help="RGB for first path (0-1 floats, comma separated)")
    parser.add_argument("--color-b", default="1.0,0.5,0.0",
                        help="RGB for second path (0-1 floats, comma separated)")
    parser.add_argument("--frame", default="world",
                        help="TF frame for visualization markers")
    parser.add_argument("--label", default="saved_rrt_path",
                        help="Namespace label for markers")
    parser.add_argument("--service", default="/quad_rrt/visualize_saved_rrt",
                        help="Service name to call on quad_rrt_node")
    args = parser.parse_args()

    files = [args.file_a]
    if args.file_b:
        files.append(args.file_b)
    file_path_arg = ",".join(files)

    color_a = parse_rgb(args.color_a)
    color_b = parse_rgb(args.color_b)

    rospy.init_node("visualize_saved_twc_client", anonymous=True)
    rospy.wait_for_service(args.service)
    srv = rospy.ServiceProxy(args.service, VisualizeStampedPoses)

    # First call for path A (and optional B) with color A
    req = VisualizeStampedPosesRequest()
    req.label = args.label
    req.file_path = file_path_arg
    req.frame = args.frame
    req.r, req.g, req.b = color_a
    req.a = 1.0

    try:
        resp = srv(req)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        sys.exit(1)

    if not resp.success:
        rospy.logerr("Visualization failed: %s", resp.message)
        sys.exit(1)

    rospy.loginfo("Visualization OK: %s", resp.message)
    rospy.loginfo("First path color: %s, second path color: %s",
                  color_a, color_b)

    # If second path provided, recolor it by calling again with only second file.
    if args.file_b:
        req.file_path = args.file_b
        req.r, req.g, req.b = color_b
        req.label = args.label + "_b"
        try:
            resp_b = srv(req)
            if resp_b.success:
                rospy.loginfo("Second path visualization OK: %s", resp_b.message)
            else:
                rospy.logwarn("Second path visualization failed: %s", resp_b.message)
        except rospy.ServiceException as e:
            rospy.logwarn("Second path service call failed: %s", e)


if __name__ == "__main__":
    main()
