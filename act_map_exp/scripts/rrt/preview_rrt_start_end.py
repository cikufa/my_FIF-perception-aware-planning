#!/usr/bin/env python2

import argparse
import glob
import math
import os
import shlex
import subprocess
import sys
import time

import yaml


def _repo_root():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(this_dir, "..", "..", ".."))


def _default_rrt_dir():
    return os.path.join(
        _repo_root(), "act_map_exp", "params", "quad_rrt", "warehouse"
    )


def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.load(f, Loader=yaml.FullLoader)


def _call(cmd_tokens, dry_run=False):
    print(">> {}".format(" ".join(cmd_tokens)))
    if dry_run:
        return 0
    return subprocess.call(cmd_tokens)


def _collect_files(dir_path, pattern, files):
    if files:
        out = []
        for f in files:
            abs_f = os.path.abspath(f)
            if os.path.isdir(abs_f):
                out.extend(sorted(glob.glob(os.path.join(abs_f, pattern))))
            else:
                out.append(abs_f)
        return sorted(out)
    return sorted(glob.glob(os.path.join(dir_path, pattern)))


def _fmt_vec(v):
    if v is None:
        return "None"
    return "[{:.3f}, {:.3f}, {:.3f}]".format(v[0], v[1], v[2])


def _wait_for_service(service_name, timeout_sec):
    start = time.time()
    while True:
        try:
            out = subprocess.check_output(["rosservice", "list"]).decode("utf-8")
        except Exception:
            out = ""
        if service_name in out.splitlines():
            return True
        if timeout_sec is not None and timeout_sec >= 0:
            if (time.time() - start) > timeout_sec:
                return False
        time.sleep(0.5)


def _list_visualize_rrt_services():
    try:
        out = subprocess.check_output(["rosservice", "list"]).decode("utf-8")
    except Exception:
        return []
    nodes = []
    for line in out.splitlines():
        line = line.strip()
        if line.endswith("/visualize_saved_rrt"):
            parts = line.split("/")
            if len(parts) >= 3 and parts[1]:
                nodes.append(parts[1])
    return sorted(list(set(nodes)))


def _select_rrt_node(available_nodes, var_name, preferred_node):
    if preferred_node in available_nodes:
        return preferred_node
    name = var_name.lower()
    base = None
    if "gp" in name:
        base = "quad_rrt_gp"
    elif "quad" in name:
        base = "quad_rrt_quadratic"

    if base:
        if "trace" in name:
            cand = base + "_trace"
            if cand in available_nodes:
                return cand
        if "det" in name or "info" in name or "none" in name or "pc" in name:
            cand = base + "_info"
            if cand in available_nodes:
                return cand
        for cand in available_nodes:
            if cand.startswith(base):
                return cand
    return available_nodes[0] if available_nodes else preferred_node


def _pause_or_interrupt(prompt, dry_run, no_pause):
    if dry_run or no_pause:
        return False
    try:
        raw_input(prompt)
        return False
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return True


def _start_voxblox(args):
    load_srv = "/voxblox_node/load_map"
    mesh_srv = "/voxblox_node/generate_mesh"

    already_up = _wait_for_service(load_srv, 0.1)
    proc = None
    if not already_up:
        launch_cmd = ["roslaunch"] + shlex.split(args.voxblox_launch)
        print(">> {}".format(" ".join(launch_cmd)))
        if not args.dry_run:
            proc = subprocess.Popen(launch_cmd)
            time.sleep(0.5)

    ok = _wait_for_service(load_srv, args.voxblox_wait_sec)
    if not ok:
        print("ERROR: Service [{}] not available.".format(load_srv))
        return proc

    if not args.voxblox_skip_load and args.voxblox_map:
        map_path = os.path.abspath(args.voxblox_map)
        if not os.path.exists(map_path):
            print("ERROR: map file does not exist: {}".format(map_path))
        else:
            _call(
                [
                    "rosservice",
                    "call",
                    load_srv,
                    "file_path: '{}'".format(map_path),
                ],
                args.dry_run,
            )

    if not args.no_mesh:
        if _wait_for_service(mesh_srv, args.voxblox_wait_sec):
            _call(["rosservice", "call", mesh_srv], args.dry_run)
        else:
            print("ERROR: Service [{}] not available.".format(mesh_srv))

    return proc


def _start_rrt(args):
    srv_name = "/{}/visualize_saved_rrt".format(args.node)
    available = _list_visualize_rrt_services()
    already_up = len(available) > 0
    proc = None
    if not already_up:
        launch_cmd = ["roslaunch"] + shlex.split(args.rrt_launch)
        print(">> {}".format(" ".join(launch_cmd)))
        if not args.dry_run:
            proc = subprocess.Popen(launch_cmd)
            time.sleep(0.5)

    if not already_up:
        ok = _wait_for_service(srv_name, args.rrt_wait_sec)
        if not ok:
            print("ERROR: Service [{}] not available.".format(srv_name))
    return proc


def _ensure_ros_node():
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node("rrt_start_end_preview", anonymous=True)


def _get_pub(pub_cache, topic, msg_type, latch=False):
    if topic in pub_cache:
        return pub_cache[topic]
    import rospy
    pub = rospy.Publisher(topic, msg_type, queue_size=1, latch=latch)
    pub_cache[topic] = pub
    return pub


def _clear_node_viz(node, frame_id, pub_cache):
    import rospy
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker, MarkerArray

    now = rospy.Time.now()
    pc = PointCloud2()
    pc.header.frame_id = frame_id
    pc.header.stamp = now
    pc.height = 0
    pc.width = 0
    pc.is_bigendian = False
    pc.is_dense = True
    pc.point_step = 0
    pc.row_step = 0
    pc.data = []

    traj_pub = _get_pub(pub_cache, "/{}/traj_pos".format(node), PointCloud2)
    traj_pub.publish(pc)

    ma = MarkerArray()
    m_clear = Marker()
    m_clear.action = Marker.DELETEALL
    ma.markers.append(m_clear)

    orient_pub = _get_pub(pub_cache, "/{}/traj_orient".format(node), MarkerArray)
    orient_pub.publish(ma)

    gm_pub = _get_pub(pub_cache, "/{}/general_markers".format(node), Marker)
    m = Marker()
    m.action = Marker.DELETEALL
    gm_pub.publish(m)

    edge_pub = _get_pub(pub_cache, "/{}/rrt_edge".format(node), Marker)
    m2 = Marker()
    m2.action = Marker.DELETEALL
    edge_pub.publish(m2)

    rospy.sleep(0.05)


def _yaw_to_quat(yaw_rad):
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _yaw_to_rot(yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _rot_mul(a, b):
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _rot_vec_mul(r, v):
    return [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]


def _rot_to_quat(r):
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    return (qx, qy, qz, qw)


def _parse_tbc(data):
    tbc = data.get("T_BC")
    if not tbc or not isinstance(tbc, list) or len(tbc) < 16:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    return [
        [float(tbc[0]), float(tbc[1]), float(tbc[2]), float(tbc[3])],
        [float(tbc[4]), float(tbc[5]), float(tbc[6]), float(tbc[7])],
        [float(tbc[8]), float(tbc[9]), float(tbc[10]), float(tbc[11])],
        [float(tbc[12]), float(tbc[13]), float(tbc[14]), float(tbc[15])],
    ]


def _build_marker_array(
    frame_id,
    start,
    end,
    start_yaw_deg,
    end_yaw_deg,
    tbc,
    scale,
    text_scale,
    z_offset,
    include_yaw,
    include_cam,
):
    import rospy
    from visualization_msgs.msg import Marker, MarkerArray

    def _color(r, g, b, a=1.0):
        return (r, g, b, a)

    def _sphere(mid, ns, pos, col):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2] + z_offset
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = col
        return m

    def _text(mid, ns, pos, txt, col):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2] + z_offset + (1.5 * scale)
        m.pose.orientation.w = 1.0
        m.scale.z = text_scale
        m.color.r, m.color.g, m.color.b, m.color.a = col
        m.text = txt
        return m

    def _arrow(mid, ns, pos, yaw_deg, col):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2] + z_offset
        if yaw_deg is not None:
            yaw_rad = math.radians(float(yaw_deg))
            qx, qy, qz, qw = _yaw_to_quat(yaw_rad)
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
        else:
            m.pose.orientation.w = 1.0
        m.scale.x = scale * 3.0
        m.scale.y = scale * 0.6
        m.scale.z = scale * 0.6
        m.color.r, m.color.g, m.color.b, m.color.a = col
        return m

    def _cam_markers(base_id, ns, pos, rot, label, col):
        # Align arrow to camera optical axis (+Z) instead of +X.
        r_align = [
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
        ]
        r_arrow = _rot_mul(rot, r_align)
        qx, qy, qz, qw = _rot_to_quat(r_arrow)
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = base_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2] + z_offset
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = col

        t = Marker()
        t.header.frame_id = frame_id
        t.header.stamp = rospy.Time.now()
        t.ns = ns
        t.id = base_id + 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = pos[0]
        t.pose.position.y = pos[1]
        t.pose.position.z = pos[2] + z_offset + (1.5 * scale)
        t.pose.orientation.w = 1.0
        t.scale.z = text_scale
        t.color.r, t.color.g, t.color.b, t.color.a = col
        t.text = label

        a = Marker()
        a.header.frame_id = frame_id
        a.header.stamp = rospy.Time.now()
        a.ns = ns
        a.id = base_id + 2
        a.type = Marker.ARROW
        a.action = Marker.ADD
        a.pose.position.x = pos[0]
        a.pose.position.y = pos[1]
        a.pose.position.z = pos[2] + z_offset
        a.pose.orientation.x = qx
        a.pose.orientation.y = qy
        a.pose.orientation.z = qz
        a.pose.orientation.w = qw
        a.scale.x = scale * 3.0
        a.scale.y = scale * 0.6
        a.scale.z = scale * 0.6
        a.color.r, a.color.g, a.color.b, a.color.a = col
        return [m, t, a]

    ma = MarkerArray()
    if start is not None:
        ma.markers.append(_sphere(0, "rrt_start", start, _color(0.1, 0.9, 0.1)))
        ma.markers.append(_text(1, "rrt_start", start, "start", _color(0.1, 0.9, 0.1)))
        if include_yaw:
            ma.markers.append(_arrow(2, "rrt_start", start, start_yaw_deg, _color(0.1, 0.9, 0.1)))
        if include_cam:
            yaw = 0.0 if start_yaw_deg is None else math.radians(float(start_yaw_deg))
            r_wb = _yaw_to_rot(yaw)
            r_bc = [
                [tbc[0][0], tbc[0][1], tbc[0][2]],
                [tbc[1][0], tbc[1][1], tbc[1][2]],
                [tbc[2][0], tbc[2][1], tbc[2][2]],
            ]
            t_bc = [tbc[0][3], tbc[1][3], tbc[2][3]]
            t_wc = _rot_vec_mul(r_wb, t_bc)
            t_wc = [start[0] + t_wc[0], start[1] + t_wc[1], start[2] + t_wc[2]]
            r_wc = _rot_mul(r_wb, r_bc)
            ma.markers.extend(
                _cam_markers(10, "rrt_start_cam", t_wc, r_wc, "start_cam", _color(0.2, 0.6, 1.0))
            )
    if end is not None:
        ma.markers.append(_sphere(3, "rrt_end", end, _color(0.9, 0.1, 0.1)))
        ma.markers.append(_text(4, "rrt_end", end, "end", _color(0.9, 0.1, 0.1)))
        if include_yaw:
            ma.markers.append(_arrow(5, "rrt_end", end, end_yaw_deg, _color(0.9, 0.1, 0.1)))
        if include_cam:
            yaw = 0.0 if end_yaw_deg is None else math.radians(float(end_yaw_deg))
            r_wb = _yaw_to_rot(yaw)
            r_bc = [
                [tbc[0][0], tbc[0][1], tbc[0][2]],
                [tbc[1][0], tbc[1][1], tbc[1][2]],
                [tbc[2][0], tbc[2][1], tbc[2][2]],
            ]
            t_bc = [tbc[0][3], tbc[1][3], tbc[2][3]]
            t_wc = _rot_vec_mul(r_wb, t_bc)
            t_wc = [end[0] + t_wc[0], end[1] + t_wc[1], end[2] + t_wc[2]]
            r_wc = _rot_mul(r_wb, r_bc)
            ma.markers.extend(
                _cam_markers(20, "rrt_end_cam", t_wc, r_wc, "end_cam", _color(0.2, 0.6, 1.0))
            )
    return ma


def _publish_markers(
    pub,
    frame_id,
    start,
    end,
    start_yaw_deg,
    end_yaw_deg,
    tbc,
    scale,
    text_scale,
    z_offset,
    include_yaw,
    include_cam,
    clear_first,
):
    import rospy
    from visualization_msgs.msg import Marker, MarkerArray

    if clear_first:
        clear_ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear_ma.markers.append(clear)
        pub.publish(clear_ma)
        rospy.sleep(0.05)

    ma = _build_marker_array(
        frame_id,
        start,
        end,
        start_yaw_deg,
        end_yaw_deg,
        tbc,
        scale,
        text_scale,
        z_offset,
        include_yaw,
        include_cam,
    )
    pub.publish(ma)
    rospy.sleep(0.05)


def _trace_none_pose_file(trace_root, base_name):
    trace_dir = os.path.join(trace_root, base_name, base_name + "_none")
    if not os.path.isdir(trace_dir):
        return None
    pose_fn = os.path.join(trace_dir, "stamped_Twc.txt")
    if os.path.isfile(pose_fn):
        return pose_fn
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Preview start/end from RRT YAMLs via rosservice calls."
    )
    parser.add_argument(
        "--dir",
        default=_default_rrt_dir(),
        help="Directory containing RRT YAML configs.",
    )
    parser.add_argument(
        "--pattern",
        default="*.yaml",
        help="Glob pattern for YAML files inside --dir.",
    )
    parser.add_argument(
        "--files",
        nargs="*",
        default=None,
        help="Optional list of YAML files or directories to use instead of --dir.",
    )
    parser.add_argument(
        "--node",
        default="quad_rrt",
        help="Planner node name (e.g. quad_rrt, quad_rrt_gp_info).",
    )
    parser.add_argument(
        "--no-mesh",
        action="store_true",
        help="Skip rosservice call /voxblox_node/generate_mesh.",
    )
    parser.add_argument(
        "--mesh-every",
        action="store_true",
        help="Call /voxblox_node/generate_mesh before each config.",
    )
    parser.add_argument(
        "--no-plan",
        action="store_true",
        help="Only print start/end; skip planner service calls.",
    )
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Skip rosservice call /<node>/reset_planner.",
    )
    parser.add_argument(
        "--no-pause",
        action="store_true",
        help="Do not pause between configs.",
    )
    parser.add_argument(
        "--wait-sec",
        type=float,
        default=0.0,
        help="Optional sleep seconds after each plan.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands but do not execute them.",
    )
    parser.add_argument(
        "--no-voxblox",
        action="store_true",
        help="Do not launch/load/generate mesh for voxblox.",
    )
    parser.add_argument(
        "--no-rrt",
        action="store_true",
        help="Do not launch quad_rrt for trace visualization.",
    )
    parser.add_argument(
        "--rrt-launch",
        default="act_map_exp quad_rrt_warehouse_quadratic.launch",
        help="roslaunch arguments for quad_rrt (package + launch file).",
    )
    parser.add_argument(
        "--rrt-wait-sec",
        type=float,
        default=15.0,
        help="Seconds to wait for quad_rrt services.",
    )
    parser.add_argument(
        "--rrt-stop",
        action="store_true",
        help="Stop roslaunch quad_rrt when script exits.",
    )
    parser.add_argument(
        "--voxblox-launch",
        default="act_map_ros voxblox_warehouse.launch",
        help="roslaunch arguments for voxblox (package + launch file).",
    )
    parser.add_argument(
        "--voxblox-map",
        default=os.path.join(
            _repo_root(),
            "act_map_exp",
            "exp_data",
            "warehouse_voxblox",
            "tsdf_esdf_max10.vxblx",
        ),
        help="Map file for /voxblox_node/load_map.",
    )
    parser.add_argument(
        "--voxblox-wait-sec",
        type=float,
        default=15.0,
        help="Seconds to wait for voxblox services.",
    )
    parser.add_argument(
        "--voxblox-skip-load",
        action="store_true",
        help="Do not call /voxblox_node/load_map.",
    )
    parser.add_argument(
        "--voxblox-stop",
        action="store_true",
        help="Stop roslaunch voxblox when script exits.",
    )
    parser.add_argument(
        "--markers",
        action="store_true",
        default=True,
        help="Publish RViz markers for start/end (default: on).",
    )
    parser.add_argument(
        "--marker-topic",
        default="rrt_start_end_markers",
        help="Marker topic name (relative to root).",
    )
    parser.add_argument(
        "--marker-frame",
        default="world",
        help="Marker frame_id.",
    )
    parser.add_argument(
        "--marker-scale",
        type=float,
        default=0.6,
        help="Marker sphere scale (meters).",
    )
    parser.add_argument(
        "--marker-text-scale",
        type=float,
        default=0.6,
        help="Marker text height (meters).",
    )
    parser.add_argument(
        "--marker-z-offset",
        type=float,
        default=0.0,
        help="Z offset for markers (meters).",
    )
    parser.add_argument(
        "--marker-no-yaw",
        action="store_true",
        help="Do not show yaw arrows.",
    )
    parser.add_argument(
        "--marker-no-cam",
        action="store_true",
        help="Do not show camera start/end markers.",
    )
    parser.add_argument(
        "--marker-no-clear",
        action="store_true",
        help="Do not clear old markers before publishing new ones.",
    )
    parser.add_argument(
        "--marker-latch",
        action="store_true",
        help="Latch marker publisher so RViz keeps the last markers.",
    )
    args = parser.parse_args()

    cfg_dir = os.path.abspath(args.dir)
    files = _collect_files(cfg_dir, args.pattern, args.files)
    excluded = set(["warehouse_all.yaml", "warehouse_mini.yaml", "warehouse_rrt_trial.yaml"])
    files = [f for f in files if os.path.basename(f) not in excluded]
    if not files:
        print("No YAML files found under {}".format(cfg_dir))
        return 1

    voxblox_proc = None
    rrt_proc = None
    exit_code = 0
    try:
        if not args.no_voxblox:
            voxblox_proc = _start_voxblox(args)

        if not args.no_rrt:
            rrt_proc = _start_rrt(args)

        marker_pub = None
        pub_cache = {}
        if args.markers:
            try:
                import rospy
                from visualization_msgs.msg import MarkerArray
            except Exception as e:
                print("Failed to import ROS markers: {}".format(e))
                return 1
            if not args.dry_run:
                _ensure_ros_node()
                marker_pub = rospy.Publisher(
                    "/" + args.marker_topic.strip("/"),
                    MarkerArray,
                    queue_size=1,
                    latch=args.marker_latch,
                )
                rospy.sleep(0.1)

        available_rrt_nodes = _list_visualize_rrt_services()

        for idx, cfg in enumerate(files):
            if not os.path.isfile(cfg):
                continue
            try:
                data = _load_yaml(cfg) or {}
            except Exception as e:
                print("Failed to load {}: {}".format(cfg, e))
                continue

            start = data.get("start")
            end = data.get("end")
            syaw = data.get("start_yaw_deg", None)
            eyaw = data.get("end_yaw_deg", None)

            if start is None or end is None:
                print("\n=== {} ({}/{}) ===".format(os.path.basename(cfg), idx + 1, len(files)))
                print("Skipping (missing start/end).")
                continue

            print("\n=== {} ({}/{}) ===".format(os.path.basename(cfg), idx + 1, len(files)))
            print("start: {}  start_yaw_deg: {}".format(_fmt_vec(start), syaw))
            print("end:   {}  end_yaw_deg:   {}".format(_fmt_vec(end), eyaw))

            if args.markers and marker_pub is not None and not args.dry_run:
                tbc = _parse_tbc(data)
                _publish_markers(
                    marker_pub,
                    args.marker_frame,
                    start,
                    end,
                    syaw,
                    eyaw,
                    tbc,
                    args.marker_scale,
                    args.marker_text_scale,
                    args.marker_z_offset,
                    not args.marker_no_yaw,
                    not args.marker_no_cam,
                    not args.marker_no_clear,
                )

            trace_visualized = False
            trace_root = os.path.join(_repo_root(), "act_map_exp", "trace")
            base_name = os.path.splitext(os.path.basename(cfg))[0]
            pose_fn = _trace_none_pose_file(trace_root, base_name)
            if not available_rrt_nodes:
                print("trace: visualize_saved_rrt not available; skipping traces")
            else:
                if not args.dry_run:
                    _ensure_ros_node()
                    for node_nm in available_rrt_nodes:
                        _clear_node_viz(node_nm, args.marker_frame, pub_cache)
                if pose_fn:
                    node_to_use = _select_rrt_node(
                        available_rrt_nodes, base_name + "_none", args.node
                    )
                    print("trace: {} -> {} ({})".format(base_name + "_none", pose_fn, node_to_use))
                    vis_cmd = [
                        "rosservice",
                        "call",
                        "/{}/visualize_saved_rrt".format(node_to_use),
                        "config: '{}'".format(os.path.abspath(pose_fn)),
                    ]
                    _call(vis_cmd, args.dry_run)
                    trace_visualized = True
                else:
                    print("trace: (no stamped_Twc.txt under {}/{})".format(base_name, base_name + "_none"))

            if exit_code != 0:
                break

            if args.no_plan:
                if _pause_or_interrupt(
                    "Press Enter for next config...",
                    args.dry_run,
                    args.no_pause,
                ):
                    exit_code = 1
                    break
                continue

            if args.mesh_every and not args.no_mesh:
                _call(["rosservice", "call", "/voxblox_node/generate_mesh"], args.dry_run)

            set_cmd = [
                "rosservice",
                "call",
                "/{}/set_planner_state".format(args.node),
                "config: '{}'".format(os.path.abspath(cfg)),
            ]
            _call(set_cmd, args.dry_run)

            _call(
                ["rosservice", "call", "/{}/plan_vis_save".format(args.node)],
                args.dry_run,
            )

            if not args.no_reset:
                _call(
                    ["rosservice", "call", "/{}/reset_planner".format(args.node)],
                    args.dry_run,
                )

            if args.wait_sec > 0:
                time.sleep(args.wait_sec)

            if _pause_or_interrupt(
                "Press Enter for next config...",
                args.dry_run,
                args.no_pause,
            ):
                exit_code = 1
                break
    finally:
        if args.voxblox_stop and voxblox_proc is not None:
            try:
                voxblox_proc.terminate()
                voxblox_proc.wait()
            except Exception:
                pass
        if args.rrt_stop and rrt_proc is not None:
            try:
                rrt_proc.terminate()
                rrt_proc.wait()
            except Exception:
                pass

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
