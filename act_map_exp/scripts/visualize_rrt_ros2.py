#!/usr/bin/env python3
"""
Trigger RViz visualization for Voxblox mesh and RRT trajectory via ROS 2 services.

This script only calls services; it does not launch planners or RViz.
"""

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Empty
except ImportError as exc:
    raise SystemExit(
        "ROS 2 Python packages are missing (rclpy/std_srvs). "
        "Source your ROS 2 workspace before running this script."
    ) from exc


class ServiceCaller(Node):
    def __init__(self, timeout_sec: float):
        super().__init__("rrt_viz_trigger")
        self.timeout_sec = float(timeout_sec)

    def wait_for(self, client, name: str) -> bool:
        if self.timeout_sec <= 0.0:
            self.get_logger().info(f"Waiting for service {name} ...")
            while not client.wait_for_service(timeout_sec=1.0):
                pass
            return True
        end_time = time.time() + self.timeout_sec
        while time.time() < end_time:
            if client.wait_for_service(timeout_sec=0.5):
                return True
        return False

    def call_empty(self, client, name: str) -> bool:
        self.get_logger().info(f"Calling service {name}")
        req = Empty.Request()
        future = client.call_async(req)
        timeout = None if self.timeout_sec <= 0.0 else self.timeout_sec
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f"Service call timed out: {name}")
            return False
        if future.exception() is not None:
            self.get_logger().error(f"Service call failed: {name}: {future.exception()}")
            return False
        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Call ROS 2 services to visualize Voxblox mesh and RRT trajectory."
    )
    parser.add_argument(
        "--mesh-service",
        default="/voxblox_node/generate_mesh",
        help="ROS 2 service to trigger mesh generation.",
    )
    parser.add_argument(
        "--traj-service",
        default="/quad_rrt/publish_traj_vis",
        help="ROS 2 service to publish trajectory visualization (no planning).",
    )
    parser.add_argument(
        "--skip-mesh",
        action="store_true",
        help="Skip mesh generation call.",
    )
    parser.add_argument(
        "--skip-traj",
        action="store_true",
        help="Skip trajectory visualization call.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Wait/call timeout in seconds (0 = wait forever).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.skip_mesh and args.skip_traj:
        print("Both mesh and trajectory calls are skipped; nothing to do.")
        return 1

    rclpy.init()
    node = ServiceCaller(args.timeout)

    try:
        if not args.skip_mesh:
            mesh_client = node.create_client(Empty, args.mesh_service)
            if not node.wait_for(mesh_client, args.mesh_service):
                node.get_logger().error(f"Service not available: {args.mesh_service}")
                return 2
            if not node.call_empty(mesh_client, args.mesh_service):
                return 3

        if not args.skip_traj:
            traj_client = node.create_client(Empty, args.traj_service)
            if not node.wait_for(traj_client, args.traj_service):
                node.get_logger().error(f"Service not available: {args.traj_service}")
                return 4
            if not node.call_empty(traj_client, args.traj_service):
                return 5
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
