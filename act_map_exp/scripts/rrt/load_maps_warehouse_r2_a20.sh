#!/usr/bin/env bash

pkg_dir=$(rospack find act_map_exp)
map_dir="${pkg_dir}/exp_data/warehouse_FIF"

rosservice call /quad_rrt_gp_info/clear_act_map_layers
rosservice call /quad_rrt_gp_info/load_act_map_layers  "file_path: '${map_dir}/gp_info_r2_a20'"

rosservice call /quad_rrt_gp_trace/clear_act_map_layers
rosservice call /quad_rrt_gp_trace/load_act_map_layers  "file_path: '${map_dir}/gp_trace_r2_a20'"

rosservice call /quad_rrt_quadratic_info/clear_act_map_layers
rosservice call /quad_rrt_quadratic_info/load_act_map_layers  "file_path: '${map_dir}/quad_info_r2_a20'"

rosservice call /quad_rrt_quadratic_trace/clear_act_map_layers
rosservice call /quad_rrt_quadratic_trace/load_act_map_layers  "file_path: '${map_dir}/quad_trace_r2_a20'"
