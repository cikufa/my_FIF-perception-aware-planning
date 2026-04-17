#!/usr/bin/env python2

import os
import time
import subprocess

import rospy


def _split_csv(value):
    if not value:
        return []
    return [v.strip() for v in value.split(',') if v.strip()]


def _wait_for(service_name, timeout):
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        return True
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for service: %s", service_name)
        return False


def _run(cmd):
    rospy.loginfo("rrt_info_viz_setup: %s", cmd)
    ret = subprocess.call(cmd, shell=True)
    if ret != 0:
        rospy.logwarn("Command failed (%d): %s", ret, cmd)
    return ret


def main():
    rospy.init_node("rrt_info_viz_setup", anonymous=True)

    wait_sec = float(rospy.get_param("~wait_sec", 2.0))
    if wait_sec > 0.0:
        time.sleep(wait_sec)

    service_wait_timeout = float(rospy.get_param("~service_wait_timeout", 30.0))

    # Voxblox service calls
    voxblox_load_map = bool(rospy.get_param("~voxblox_load_map", False))
    voxblox_map_file = rospy.get_param("~voxblox_map_file", "")
    publish_esdf = bool(rospy.get_param("~publish_esdf", True))
    generate_mesh = bool(rospy.get_param("~generate_mesh", True))

    if voxblox_load_map:
        if not voxblox_map_file:
            rospy.logwarn("voxblox_load_map is true but voxblox_map_file is empty.")
        else:
            if _wait_for("/voxblox_node/load_map", service_wait_timeout):
                _run("rosservice call /voxblox_node/load_map \"file_path: '{}'\"".format(
                    voxblox_map_file))

    if publish_esdf:
        if _wait_for("/voxblox_node/publish_map", service_wait_timeout):
            _run("rosservice call /voxblox_node/publish_map")

    if generate_mesh:
        if _wait_for("/voxblox_node/generate_mesh", service_wait_timeout):
            _run("rosservice call /voxblox_node/generate_mesh")

    # Load RRT info maps into their servers
    load_rrt_maps = bool(rospy.get_param("~load_rrt_maps", False))
    map_root = rospy.get_param("~map_root", "")
    map_suffix = rospy.get_param("~map_suffix", "")
    rrt_nodes = _split_csv(rospy.get_param(
        "~rrt_nodes",
        "quad_rrt_gp_info,quad_rrt_gp_trace,quad_rrt_quadratic_info,quad_rrt_quadratic_trace"))
    rrt_prefixes = _split_csv(rospy.get_param(
        "~rrt_map_prefixes",
        "gp_info,gp_trace,quad_info,quad_trace"))

    if load_rrt_maps:
        if not map_root or not map_suffix:
            rospy.logwarn("load_rrt_maps is true but map_root or map_suffix is empty.")
        else:
            if not os.path.isdir(map_root):
                rospy.logwarn("map_root does not exist: %s", map_root)
            n = min(len(rrt_nodes), len(rrt_prefixes))
            if n == 0:
                rospy.logwarn("No RRT nodes or map prefixes provided.")
            if len(rrt_nodes) != len(rrt_prefixes):
                rospy.logwarn("rrt_nodes and rrt_map_prefixes length mismatch; using first %d", n)
            for node, prefix in zip(rrt_nodes[:n], rrt_prefixes[:n]):
                clear_srv = "/{}/clear_act_map_layers".format(node)
                load_srv = "/{}/load_act_map_layers".format(node)
                map_path = os.path.join(map_root, "{}_{}".format(prefix, map_suffix))
                if _wait_for(clear_srv, service_wait_timeout):
                    _run("rosservice call {}".format(clear_srv))
                if _wait_for(load_srv, service_wait_timeout):
                    _run("rosservice call {} \"file_path: '{}'\"".format(load_srv, map_path))


if __name__ == "__main__":
    main()
