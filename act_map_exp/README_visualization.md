# Visualization Readme

This document explains what `fov_debug_viz.launch` and `rrt_info_viz.launch` are intended to show in RViz, and lists all arguments/parameters used by those visualizations.

## fov_debug_viz.launch

### Purpose
`fov_debug_viz.launch` runs `visualize_fov_debug.py`, which animates per-iteration camera quivers and overlays per-pose visibility diagnostics computed from a point set. It is intended for inspecting FoV optimization outputs.

### Inputs
- Quivers file (required): blocks separated by blank lines; each line is `x,y,z,dx,dy,dz`.
- Points file (optional): XYZ columns; can be a CSV/space-separated file or COLMAP `points3D.txt` with configurable columns.

### RViz Outputs (MarkerArray on `/fov_debug/markers`)
- Path: `LINE_STRIP` showing the pose sequence for the current iteration.
- View arrows: `LINE_LIST` showing the camera direction at each pose.
- Pose spheres: `SPHERE_LIST` colored by visibility count/score.
- Optional per-pose text: `TEXT_VIEW_FACING` labels (count/score).
- Optional visibility links: `LINE_LIST` from a selected pose to visible points.
- Optional visible points: `POINTS` for the selected pose.
- Status text: `TEXT_VIEW_FACING` with iteration and min/mean/max.

Marker namespaces are prefixed by the `--namespace` argument (default `fov_debug`), e.g. `fov_debug_path`, `fov_debug_view`, `fov_debug_pose`, `fov_debug_text`, `fov_debug_links`, `fov_debug_vis_pts`, `fov_debug_status`.

### Launch Arguments
These are the args defined in `launch/fov_debug_viz.launch`.

| Launch Arg | Default | Meaning |
| --- | --- | --- |
| `quivers` | `""` | Path to quivers file (required by the script). |
| `points` | `""` | Optional points file for visibility scoring. |
| `points_cols` | `0,1,2` | XYZ column indices in the points file (0-based). |
| `frame` | `world` | RViz fixed frame. |
| `topic` | `fov_debug/markers` | MarkerArray topic. |
| `rate` | `2.0` | Playback rate (Hz). |
| `fov_deg` | `90.0` | Horizontal FOV in degrees. |
| `pose_index` | `-1` | If >= 0, show links/points for this pose only. |
| `extra_args` | `""` | Extra CLI args forwarded to the script. |

### Script Arguments (Full List)
These are all arguments supported by `scripts/visualize_fov_debug.py`. Use `extra_args` in the launch file to set any that are not exposed directly.

| Arg | Default | Meaning |
| --- | --- | --- |
| `--quivers` | required | Quivers file path. |
| `--points` | `""` | Points file path (optional). |
| `--points-cols` | `0,1,2` | XYZ column indices (0-based). |
| `--frame` | `world` | RViz fixed frame. |
| `--topic` | `fov_debug/markers` | MarkerArray topic. |
| `--namespace` | `fov_debug` | Marker namespace prefix. |
| `--rate` | `2.0` | Playback rate in Hz. |
| `--start-iter` | `0` | First iteration to display. |
| `--end-iter` | `-1` | Last iteration to display (`-1` = final). |
| `--no-loop` | false | Play once and stop. |
| `--fov-deg` | `90.0` | Horizontal FOV in degrees. |
| `--min-range` | `0.0` | Minimum visibility range (m). |
| `--max-range` | `-1.0` | Maximum visibility range (m), `-1` disables. |
| `--use-sigmoid` | false | Use sigmoid visibility score instead of hard counts. |
| `--sigmoid-k` | `15.0` | Sigmoid sharpness. |
| `--pose-index` | `-1` | Pose index for links/visible points. |
| `--show-links` | false | Show links from pose to visible points. |
| `--show-visible-points` | false | Show visible points for pose. |
| `--links-every` | `5` | Downsample visibility links (every Nth). |
| `--point-stride` | `4` | Stride for points subsampling. |
| `--max-points` | `8000` | Max points loaded. |
| `--arrow-scale` | `0.05` | Arrow length as ratio of path diagonal. |
| `--path-width-scale` | `0.006` | Path width as ratio of path diagonal. |
| `--pose-scale` | `0.03` | Pose sphere diameter as ratio of path diagonal. |
| `--text-every` | `5` | Show count text every N poses (0 disables). |

### Example
```bash
roslaunch act_map_exp fov_debug_viz.launch \
  quivers:=/abs/path/quivers_path_yaw.txt \
  points:=/abs/path/points.csv \
  points_cols:=0,1,2 \
  extra_args:="--show-links --show-visible-points --pose-index 10"
```

## rrt_info_viz.launch

### Purpose
`rrt_info_viz.launch` runs the `rrt_info_viz_node` C++ node to visualize an RRT trajectory/path with information-field metrics along the path, plus optional voxel heatmaps of the information map. It can also auto-load maps and start RViz.

### RViz Outputs (Topics)
All topics are under the node namespace `/rrt_info_viz` (private node handle):
- `/rrt_info_viz/heatmap` (`Marker`, `SPHERE_LIST`): path poses colored by info metric value.
- `/rrt_info_viz/general_markers` (`Marker`): the path line (`rrt_path`).
- `/rrt_info_viz/camera_marker` (`MarkerArray`): camera frusta/axes along the path.
- `/rrt_info_viz/map_heatmap` (`Marker`, `CUBE_LIST`): voxel heatmap (static or follow).
- `/rrt_info_viz/value_text` (`Marker`): optional text labels for metric values.

The default RViz config `rviz_cfgs/rrt_info_viz.rviz` includes displays for the above topics and also `/fov_debug/markers`.

### Required Inputs
- `traj_file` must point to a saved trajectory/path file.
- Map layers must exist under `map_root` (or `map_dir`) with `occ_layer.protobuf` and `<map_type>_layer.protobuf`.

`map_type` and `info_metric_type` can be `auto`. When `auto` is used, the node tries to infer them from the trajectory filename or its parent directory, using known prefixes like `gp_trace`, `gp_info`, `quad_trace`, `quad_info`, `pc_trace`, `pc_info`.

### Launch Arguments
These are the args defined in `launch/rrt_info_viz.launch`.

| Launch Arg | Default | Meaning |
| --- | --- | --- |
| `traj_file` | `""` | Trajectory/path file (required). |
| `traj_format` | `auto` | Trajectory format; inferred if `auto`. |
| `map_type` | `auto` | Map type (`gp_*`, `quad_*`, `pc_*`) or inferred if `auto`. |
| `map_root` | `$(find act_map_exp)/exp_data/warehouse_FIF` | Root folder for map layers. |
| `map_suffix` | `r2_a20` | Suffix appended to map type to build map folder name. |
| `map_dir` | `""` | Explicit map folder path (overrides `map_root` + `map_suffix`). |
| `act_map_params` | `$(find act_map_ros)/params/act_map_warehouse.yaml` | ActMap parameters YAML. |
| `gp_vis_dir` | `$(find act_map)/params/fov_approximator_gp/fov45_fs70_lm1000_k15` | GP visibility approximator params. |
| `info_metric_type` | `auto` | Info metric type (`trace`, `det`, `info`) or inferred. |
| `playback_hz` | `2.0` | Playback rate. |
| `heatmap_mode` | `visited` | `visited` (up to current pose) or `all` (entire path). |
| `heatmap_scale` | `0.6` | Heatmap sphere size. |
| `heatmap_alpha` | `1.0` | Heatmap transparency. |
| `heatmap_stride` | `1` | Subsample path points for heatmap. |
| `publish_heatmap` | `true` | Publish path heatmap. |
| `publish_path` | `true` | Publish the path line. |
| `publish_camera` | `true` | Publish camera markers. |
| `publish_map_heatmap` | `true` | Publish map heatmap. |
| `map_heatmap_mode` | `follow` | `static` or `follow` (updates at current pose). |
| `map_heatmap_stride` | `1` | Subsample voxel centers. |
| `map_heatmap_alpha` | `0.03` | Map heatmap transparency. |
| `map_heatmap_vox_scale` | `1.0` | Scale factor for voxel marker size. |
| `map_heatmap_update_every_n` | `1` | Update rate for follow mode. |
| `map_heatmap_visible_only` | `false` | Only voxels visible from current pose. |
| `map_heatmap_use_z_slice` | `false` | Use a fixed Z slice only. |
| `map_heatmap_z` | `2.0` | Z value for slice. |
| `map_heatmap_z_tol` | `0.25` | Z slice tolerance. |
| `manual_mode` | `false` | If true, only update on service calls. |
| `start_paused` | `false` | Start paused. |
| `camera_marker_scale` | `2.5` | Scale for camera markers. |
| `launch_rviz` | `true` | Launch RViz with config. |
| `rviz_cfg` | `$(find act_map_exp)/rviz_cfgs/rrt_info_viz.rviz` | RViz config file. |
| `call_generate_mesh` | `false` | Call `/voxblox_node/generate_mesh`. |
| `generate_mesh_service` | `/voxblox_node/generate_mesh` | Service name for mesh generation. |
| `generate_mesh_delay` | `2.0` | Delay before calling mesh service. |
| `generate_mesh_wait_timeout` | `30.0` | Wait timeout for mesh service. |
| `run_setup` | `true` | Run `rrt_info_viz_setup.py`. |
| `setup_wait_sec` | `2.0` | Delay before setup actions. |
| `setup_service_wait_timeout` | `30.0` | Wait timeout for setup services. |
| `setup_voxblox_load_map` | `false` | Load Voxblox map file. |
| `setup_voxblox_map_file` | `""` | Voxblox map file path. |
| `setup_publish_esdf` | `true` | Call `/voxblox_node/publish_map`. |
| `setup_generate_mesh` | `true` | Call `/voxblox_node/generate_mesh`. |
| `setup_load_rrt_maps` | `true` | Load RRT info maps into servers. |
| `setup_rrt_nodes` | `quad_rrt_gp_info,quad_rrt_gp_trace,quad_rrt_quadratic_info,quad_rrt_quadratic_trace` | RRT node names for map loading. |
| `setup_rrt_map_prefixes` | `gp_info,gp_trace,quad_info,quad_trace` | Map prefixes corresponding to the RRT nodes. |

### Additional Node Parameters (Not Exposed in the Launch File)
You can set these via ROS params if needed.

| Param | Default | Meaning |
| --- | --- | --- |
| `frame_id` | `world` | RViz fixed frame. |
| `loop` | `true` | Loop playback. |
| `publish_value_text` | `false` | Publish value text markers. |
| `text_scale` | `0.6` | Text size for values. |
| `text_z_offset` | `0.5` | Z offset for value text. |
| `calculate_info_from_pc` | `false` | Use point-cloud based info queries if available. |
| `info_metric_use_log_det` | `true` | Use log-det for determinant metric. |
| `use_color_bounds` | `false` | Use manual color bounds. |
| `color_min` | `0.0` | Min value for manual color bounds. |
| `color_max` | `1.0` | Max value for manual color bounds. |

### Services (Manual Mode)
When `manual_mode=true`, the node exposes:
- `/rrt_info_viz/next_pose`
- `/rrt_info_viz/prev_pose`
- `/rrt_info_viz/reset_pose`
- `/rrt_info_viz/set_paused`

### Example
```bash
roslaunch act_map_exp rrt_info_viz.launch \
  traj_file:=/abs/path/stamped_Twc_path_yaw.txt \
  map_suffix:=r2_a20 \
  heatmap_mode:=visited \
  publish_map_heatmap:=true
```
