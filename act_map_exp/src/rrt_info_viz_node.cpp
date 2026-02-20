#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <cctype>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <yaml-cpp/yaml.h>

#include <rpg_common/main.h>
#include <rpg_common/fs.h>
#include <rpg_common/pose.h>

#include <rpg_common_ros/params_helper.h>

#include <unrealcv_bridge/ue_utils.hpp>

#include "act_map/act_map.h"
#include "act_map/info_utils.h"
#include "act_map/pos_factor_layer_evaluator.h"
#include "act_map/quadratic_vis_score.h"
#include "act_map_exp/viz_utils.h"
#include "act_map_ros/common_ros.h"
#include "act_map/voxblox/core/common.h"

namespace
{
struct Trajectory
{
  std::vector<double> times;
  rpg::PoseVec poses;
};

template <typename T>
T yamlOr(const YAML::Node& node, const std::string& key, const T& def_val)
{
  if (node[key])
  {
    return node[key].as<T>();
  }
  return def_val;
}

bool parseNumbers(const std::string& line, std::vector<double>* vals)
{
  CHECK_NOTNULL(vals);
  vals->clear();
  std::istringstream iss(line);
  double v = 0.0;
  while (iss >> v)
  {
    vals->push_back(v);
  }
  return !vals->empty();
}

bool loadTwcFile(const std::string& path, Trajectory* traj)
{
  CHECK_NOTNULL(traj);
  if (!rpg::fs::fileExists(path))
  {
    LOG(ERROR) << "Trajectory file does not exist: " << path;
    return false;
  }
  std::ifstream in(path);
  if (!in.is_open())
  {
    LOG(ERROR) << "Failed to open trajectory file: " << path;
    return false;
  }

  traj->times.clear();
  traj->poses.clear();

  std::string line;
  std::vector<double> vals;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    if (!parseNumbers(line, &vals))
    {
      continue;
    }
    const bool has_time = (vals.size() == 17u);
    const size_t offset = has_time ? 1u : 0u;
    if (vals.size() < 16u + offset)
    {
      continue;
    }

    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    size_t idx = 0;
    for (int ri = 0; ri < 4; ri++)
    {
      for (int ci = 0; ci < 4; ci++)
      {
        Twc(ri, ci) = vals[offset + idx++];
      }
    }

    rpg::Pose pose(Twc);
    traj->poses.emplace_back(pose);
    if (has_time)
    {
      traj->times.emplace_back(vals[0]);
    }
  }

  if (traj->poses.empty())
  {
    LOG(ERROR) << "No valid Twc entries found in " << path;
    return false;
  }
  return true;
}

bool loadUEPoseFile(const std::string& path, Trajectory* traj)
{
  CHECK_NOTNULL(traj);
  if (!rpg::fs::fileExists(path))
  {
    LOG(ERROR) << "Trajectory file does not exist: " << path;
    return false;
  }
  std::ifstream in(path);
  if (!in.is_open())
  {
    LOG(ERROR) << "Failed to open trajectory file: " << path;
    return false;
  }

  traj->times.clear();
  traj->poses.clear();

  std::string line;
  std::vector<double> vals;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    if (!parseNumbers(line, &vals))
    {
      continue;
    }
    const bool has_time = (vals.size() == 7u);
    const size_t offset = has_time ? 1u : 0u;
    if (vals.size() < 6u + offset)
    {
      continue;
    }

    unrealcv_bridge::UEPose uep;
    uep.x = vals[offset + 0];
    uep.y = vals[offset + 1];
    uep.z = vals[offset + 2];
    uep.pitch = vals[offset + 3];
    uep.yaw = vals[offset + 4];
    uep.roll = vals[offset + 5];

    rpg::Pose Twc;
    uep.toTwc(&Twc);
    traj->poses.emplace_back(Twc);
    if (has_time)
    {
      traj->times.emplace_back(vals[0]);
    }
  }

  if (traj->poses.empty())
  {
    LOG(ERROR) << "No valid UE poses found in " << path;
    return false;
  }
  return true;
}

enum class TrajFormat
{
  kAuto,
  kTwc,
  kUE
};

TrajFormat parseTrajFormat(const std::string& s)
{
  if (s == "twc")
  {
    return TrajFormat::kTwc;
  }
  if (s == "ue")
  {
    return TrajFormat::kUE;
  }
  return TrajFormat::kAuto;
}

bool detectFormatFromFile(const std::string& path, TrajFormat* format)
{
  CHECK_NOTNULL(format);
  std::ifstream in(path);
  if (!in.is_open())
  {
    return false;
  }
  std::string line;
  std::vector<double> vals;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    if (!parseNumbers(line, &vals))
    {
      continue;
    }
    if (vals.size() == 6u || vals.size() == 7u)
    {
      *format = TrajFormat::kUE;
      return true;
    }
    if (vals.size() == 16u || vals.size() == 17u)
    {
      *format = TrajFormat::kTwc;
      return true;
    }
    return false;
  }
  return false;
}

bool loadTrajectory(const std::string& path, TrajFormat format, Trajectory* traj)
{
  CHECK_NOTNULL(traj);
  if (format == TrajFormat::kAuto)
  {
    TrajFormat detected = TrajFormat::kAuto;
    if (!detectFormatFromFile(path, &detected))
    {
      LOG(ERROR) << "Failed to detect trajectory format for " << path;
      return false;
    }
    format = detected;
  }

  if (format == TrajFormat::kUE)
  {
    return loadUEPoseFile(path, traj);
  }
  return loadTwcFile(path, traj);
}

act_map::ActMapOptions readActMapOptionsYaml(const std::string& fn)
{
  YAML::Node node = YAML::LoadFile(fn);
  act_map::ActMapOptions opts;
  opts.occ_layer_options_.vox_size =
      yamlOr<double>(node, "occ_vox_size", 0.2);
  opts.occ_layer_options_.vox_per_side =
      yamlOr<int>(node, "occ_vox_per_side", 16);
  opts.pos_factor_layer_options_.vox_size =
      yamlOr<double>(node, "ker_vox_size", 0.2);
  opts.pos_factor_layer_options_.vox_per_side =
      yamlOr<int>(node, "ker_vox_per_side", 16);

  opts.occ_integrator_options_.probability_hit =
      yamlOr<double>(node, "occ_prob_hit", 0.65);
  opts.occ_integrator_options_.probability_miss =
      yamlOr<double>(node, "occ_prob_miss", 0.4);
  opts.occ_integrator_options_.threshold_min =
      yamlOr<double>(node, "occ_thresh_min", 0.12);
  opts.occ_integrator_options_.threshold_max =
      yamlOr<double>(node, "occ_thresh_max", 0.97);
  opts.occ_integrator_options_.threshold_occupancy =
      yamlOr<double>(node, "occ_thresh_occupancy", 0.7);
  opts.occ_integrator_options_.min_ray_length_m =
      yamlOr<double>(node, "occ_min_ray_m", 0.1);
  opts.occ_integrator_options_.max_ray_length_m =
      yamlOr<double>(node, "occ_max_ray_m", 5.0);

  act_map::QuadVisScoreOptions vis;
  vis.half_fov_rad = yamlOr<double>(node, "vis_half_fov_rad0", M_PI_4);
  vis.boundary_to_mid_ratio =
      yamlOr<double>(node, "vis_boundary_ratio0", 0.5);
  vis.boundary_value =
      yamlOr<double>(node, "vis_boundary_val0", 0.5);
  opts.vis_options_.push_back(vis);

  opts.pos_fac_integrator_options_.occ_thresh_ =
      yamlOr<double>(node, "ker_inte_occ_thresh", 0.7);

  opts.vis_checker_options_.min_dist =
      yamlOr<double>(node, "vis_check_min_dist", 0.7);
  opts.vis_checker_options_.max_dist =
      yamlOr<double>(node, "vis_check_max_dist", 10.0);
  opts.vis_checker_options_.use_view_filtering =
      yamlOr<bool>(node, "vis_check_use_view_filter", false);
  const double max_ang_deg =
      yamlOr<double>(node, "vis_check_max_ang_deg", 180.0);
  opts.vis_checker_options_.min_view_angle_cos =
      std::cos(max_ang_deg * M_PI / 180.0);

  opts.vis_checker_options_.use_depth_layer_ =
      yamlOr<bool>(node, "vis_check_use_depth_layer", false);
  opts.vis_checker_options_.depth_layer_proto_fn_ =
      yamlOr<std::string>(node, "vis_check_depth_layer_proto_fn",
                          std::string(""));
  opts.vis_checker_options_.dm_options_.depth_layer_opts_.vox_size =
      yamlOr<double>(node, "depth_layer_vox_size", 0.2);
  opts.vis_checker_options_.dm_options_.depth_layer_opts_.vox_per_side =
      yamlOr<int>(node, "depth_layer_vox_per_side", 16);
  opts.vis_checker_options_.dm_options_.depth_voxel_step_deg_ =
      yamlOr<double>(node, "depth_vox_deg_step", -1.0);

  opts.vis_checker_options_.use_camera_ =
      yamlOr<bool>(node, "vis_check_use_camera", false);
  opts.vis_checker_options_.cam_dir_ =
      yamlOr<std::string>(node, "vis_check_cam_dir", std::string(""));

  opts.use_collision_checker_ =
      yamlOr<bool>(node, "use_collision_checker", false);
  opts.col_ops_.min_dist_thresh_ =
      yamlOr<double>(node, "occ_min_dist_thresh", 0.1);
  opts.col_ops_.average_dist_thresh =
      yamlOr<double>(node, "occ_average_dist_thresh", 0.2);

  opts.eval_method = act_map::EvaluateStrategy::kInterpolation;
  return opts;
}

double clamp01(const double v)
{
  if (v < 0.0)
  {
    return 0.0;
  }
  if (v > 1.0)
  {
    return 1.0;
  }
  return v;
}

std_msgs::ColorRGBA jetColor(const double t, const double alpha)
{
  std_msgs::ColorRGBA c;
  const double x = clamp01(t);
  const double r = std::max(0.0, std::min(1.0, 1.5 - std::fabs(4.0 * x - 3.0)));
  const double g = std::max(0.0, std::min(1.0, 1.5 - std::fabs(4.0 * x - 2.0)));
  const double b = std::max(0.0, std::min(1.0, 1.5 - std::fabs(4.0 * x - 1.0)));
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = alpha;
  return c;
}

std_msgs::ColorRGBA grayColor(const double alpha)
{
  std_msgs::ColorRGBA c;
  c.r = 0.6;
  c.g = 0.6;
  c.b = 0.6;
  c.a = alpha;
  return c;
}

template <typename VoxelT>
void collectLayerCenters(const act_map::voxblox::Layer<VoxelT>& layer,
                         const int stride,
                         std::vector<Eigen::Vector3d>* centers)
{
  CHECK_NOTNULL(centers);
  centers->clear();
  act_map::voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);
  const size_t vps = layer.voxels_per_side();
  const size_t num_voxels_per_block = vps * vps * vps;
  const size_t step = std::max(1, stride);
  for (const act_map::voxblox::BlockIndex& index : blocks)
  {
    const act_map::voxblox::Block<VoxelT>& block =
        layer.getBlockByIndex(index);
    for (size_t lin_idx = 0; lin_idx < num_voxels_per_block; lin_idx += step)
    {
      const act_map::voxblox::Point coord =
          block.computeCoordinatesFromLinearIndex(lin_idx);
      centers->emplace_back(coord.x(), coord.y(), coord.z());
    }
  }
}

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool endsWith(const std::string& s, const std::string& suf)
{
  if (s.size() < suf.size())
  {
    return false;
  }
  return s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

std::string parentDirName(const std::string& path)
{
  if (path.empty())
  {
    return std::string();
  }
  const size_t end = path.find_last_not_of('/');
  if (end == std::string::npos)
  {
    return std::string();
  }
  const size_t last_sep = path.find_last_of('/', end);
  if (last_sep == std::string::npos || last_sep == 0u)
  {
    return std::string();
  }
  const size_t parent_end = last_sep - 1;
  const size_t parent_sep = path.find_last_of('/', parent_end);
  const size_t parent_start = (parent_sep == std::string::npos) ? 0u
                                                                : parent_sep + 1;
  return path.substr(parent_start, parent_end - parent_start + 1);
}

}  // namespace

class RRTInfoViz
{
public:
  RRTInfoViz(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
  {
    traj_file_ =
        rpg_ros::param<std::string>(pnh_, "traj_file", std::string(""));
    traj_format_str_ =
        rpg_ros::param<std::string>(pnh_, "traj_format", std::string("auto"));
    map_type_ =
        rpg_ros::param<std::string>(pnh_, "map_type", std::string("auto"));
    metric_str_ =
        rpg_ros::param<std::string>(pnh_, "info_metric_type", std::string("auto"));
    act_map_params_ =
        rpg_ros::param<std::string>(pnh_, "act_map_params", std::string(""));
    gp_vis_dir_ =
        rpg_ros::param<std::string>(pnh_, "gp_vis_dir", std::string(""));
    map_root_ =
        rpg_ros::param<std::string>(pnh_, "map_root", std::string(""));
    map_suffix_ =
        rpg_ros::param<std::string>(pnh_, "map_suffix", std::string("r2_a20"));
    map_dir_ =
        rpg_ros::param<std::string>(pnh_, "map_dir", std::string(""));

    frame_id_ =
        rpg_ros::param<std::string>(pnh_, "frame_id", std::string("world"));

    playback_hz_ = rpg_ros::param<double>(pnh_, "playback_hz", 2.0);
    loop_ = rpg_ros::param<bool>(pnh_, "loop", true);
    manual_mode_ = rpg_ros::param<bool>(pnh_, "manual_mode", false);
    paused_ = rpg_ros::param<bool>(pnh_, "start_paused", false);

    heatmap_mode_ =
        rpg_ros::param<std::string>(pnh_, "heatmap_mode", std::string("visited"));
    heatmap_scale_ = rpg_ros::param<double>(pnh_, "heatmap_scale", 0.2);
    heatmap_alpha_ = rpg_ros::param<double>(pnh_, "heatmap_alpha", 0.9);
    heatmap_stride_ = rpg_ros::param<int>(pnh_, "heatmap_stride", 1);
    publish_heatmap_ = rpg_ros::param<bool>(pnh_, "publish_heatmap", true);

    use_color_bounds_ =
        rpg_ros::param<bool>(pnh_, "use_color_bounds", false);
    color_min_ = rpg_ros::param<double>(pnh_, "color_min", 0.0);
    color_max_ = rpg_ros::param<double>(pnh_, "color_max", 1.0);

    publish_path_ = rpg_ros::param<bool>(pnh_, "publish_path", true);
    publish_camera_ = rpg_ros::param<bool>(pnh_, "publish_camera", true);
    publish_value_text_ =
        rpg_ros::param<bool>(pnh_, "publish_value_text", false);
    text_scale_ = rpg_ros::param<double>(pnh_, "text_scale", 0.6);
    text_z_offset_ = rpg_ros::param<double>(pnh_, "text_z_offset", 0.5);
    camera_marker_scale_ =
        rpg_ros::param<double>(pnh_, "camera_marker_scale", 0.5);

    use_info_from_pc_ =
        rpg_ros::param<bool>(pnh_, "calculate_info_from_pc", false);
    const bool use_log_det =
        rpg_ros::param<bool>(pnh_, "info_metric_use_log_det", true);
    act_map::kInfoMetricUseLogDet = use_log_det;

    publish_map_heatmap_ =
        rpg_ros::param<bool>(pnh_, "publish_map_heatmap", false);
    map_heatmap_mode_ =
        rpg_ros::param<std::string>(pnh_, "map_heatmap_mode",
                                    std::string("static"));
    map_heatmap_stride_ =
        rpg_ros::param<int>(pnh_, "map_heatmap_stride", 3);
    map_heatmap_alpha_ =
        rpg_ros::param<double>(pnh_, "map_heatmap_alpha", 0.8);
    map_heatmap_vox_scale_ =
        rpg_ros::param<double>(pnh_, "map_heatmap_vox_scale", 0.9);
    map_heatmap_update_every_n_ =
        rpg_ros::param<int>(pnh_, "map_heatmap_update_every_n", 5);
    map_heatmap_use_z_slice_ =
        rpg_ros::param<bool>(pnh_, "map_heatmap_use_z_slice", false);
    map_heatmap_z_ = rpg_ros::param<double>(pnh_, "map_heatmap_z", 2.0);
    map_heatmap_z_tol_ =
        rpg_ros::param<double>(pnh_, "map_heatmap_z_tol", 0.25);
    map_heatmap_visible_only_ =
        rpg_ros::param<bool>(pnh_, "map_heatmap_visible_only", false);

    if (traj_file_.empty())
    {
      LOG(ERROR) << "Parameter '~traj_file' is required.";
      initialized_ = false;
      return;
    }

    initDefaults();
    initialized_ = init();
  }

  bool ok() const
  {
    return initialized_;
  }

private:
  void initDefaults()
  {
    if (map_root_.empty())
    {
      const std::string exp_root = ros::package::getPath("act_map_exp");
      if (!exp_root.empty())
      {
        map_root_ = exp_root + "/exp_data/warehouse_FIF";
      }
    }
    if (act_map_params_.empty())
    {
      const std::string act_map_ros_root = ros::package::getPath("act_map_ros");
      if (!act_map_ros_root.empty())
      {
        act_map_params_ = act_map_ros_root + "/params/act_map_warehouse.yaml";
      }
    }
    if (gp_vis_dir_.empty())
    {
      const std::string act_map_root = ros::package::getPath("act_map");
      if (!act_map_root.empty())
      {
        gp_vis_dir_ = act_map_root +
                      "/params/fov_approximator_gp/fov45_fs70_lm1000_k15";
      }
    }
  }

  bool init()
  {
    if (!inferMapTypeAndMetricFromTraj())
    {
      return false;
    }
    TrajFormat format = parseTrajFormat(traj_format_str_);
    if (!loadTrajectory(traj_file_, format, &traj_))
    {
      return false;
    }
    LOG(INFO) << "Loaded trajectory with " << traj_.poses.size() << " poses.";

    if (map_dir_.empty())
    {
      map_dir_ = map_root_ + "/" + map_type_ + "_" + map_suffix_;
      map_dir_from_root_ = true;
    }
    else
    {
      map_dir_from_root_ = false;
    }

    if (!rpg::fs::fileExists(act_map_params_))
    {
      LOG(ERROR) << "act_map_params does not exist: " << act_map_params_;
      return false;
    }
    act_map_opts_ = readActMapOptionsYaml(act_map_params_);

    if (act_map::kNameToInfoMetric.count(metric_str_))
    {
      metric_type_ = act_map::kNameToInfoMetric[metric_str_];
    }
    else
    {
      LOG(WARNING) << "Unknown info_metric_type '" << metric_str_
                   << "', falling back to trace.";
      metric_type_ = act_map::InfoMetricType::kTrace;
    }
    if (isTraceMap(map_type_) && metric_type_ != act_map::InfoMetricType::kTrace)
    {
      LOG(WARNING) << "Trace map selected, forcing info metric to trace.";
      metric_type_ = act_map::InfoMetricType::kTrace;
    }

    if (!maybeFallbackPcMap())
    {
      return false;
    }

    if (!computeValues())
    {
      return false;
    }
    computeColorRange();

    heatmap_pub_ =
        pnh_.advertise<visualization_msgs::Marker>("heatmap", 5, true);
    map_heatmap_pub_ =
        pnh_.advertise<visualization_msgs::Marker>("map_heatmap", 1, true);
    path_pub_ =
        pnh_.advertise<visualization_msgs::Marker>("general_markers", 2, true);
    cam_pub_ =
        pnh_.advertise<visualization_msgs::MarkerArray>("camera_marker", 2);
    text_pub_ =
        pnh_.advertise<visualization_msgs::Marker>("value_text", 2);

    if (publish_path_)
    {
      publishPath();
    }
    if (publish_heatmap_)
    {
      const size_t upto = (heatmap_mode_ == "all") ? traj_.poses.size() - 1
                                                   : 0u;
      publishHeatmap(upto);
    }
    if (publish_camera_)
    {
      publishCamera(0u);
    }
    if (publish_value_text_)
    {
      publishValueText(0u);
    }
    if (publish_map_heatmap_)
    {
      if (map_heatmap_visible_only_)
      {
        if (map_visible_ready_)
        {
          publishMapHeatmap(traj_.poses.front());
        }
        else
        {
          LOG(WARNING)
              << "Map heatmap requested but visibility cache not ready.";
        }
      }
      else if (!map_centers_.empty())
      {
        publishMapHeatmap(traj_.poses.front());
      }
      else
      {
        LOG(WARNING) << "Map heatmap requested but no centers found.";
      }
    }

    next_pose_srv_ =
        pnh_.advertiseService("next_pose", &RRTInfoViz::nextPoseCallback, this);
    prev_pose_srv_ =
        pnh_.advertiseService("prev_pose", &RRTInfoViz::prevPoseCallback, this);
    reset_pose_srv_ =
        pnh_.advertiseService("reset_pose", &RRTInfoViz::resetPoseCallback, this);
    set_paused_srv_ =
        pnh_.advertiseService("set_paused", &RRTInfoViz::setPausedCallback, this);

    if (playback_hz_ > 0.0 && !manual_mode_)
    {
      timer_ = nh_.createTimer(ros::Duration(1.0 / playback_hz_),
                               &RRTInfoViz::timerCallback, this);
    }
    return true;
  }

  bool inferMapTypeAndMetricFromTraj()
  {
    const std::string traj_lower = toLower(traj_file_);
    std::string inferred_map;
    if (map_type_.empty() || map_type_ == "auto")
    {
      const std::vector<std::string> map_types = {
          "gp_trace", "gp_det", "gp_info",
          "quad_trace", "quad_det", "quad_info",
          "pc_trace", "pc_det", "pc_info"};
      for (const auto& mt : map_types)
      {
        if (traj_lower.find(mt) != std::string::npos)
        {
          inferred_map = mt;
          break;
        }
      }
      if (inferred_map.empty())
      {
        const std::string parent = toLower(parentDirName(traj_file_));
        for (const auto& mt : map_types)
        {
          if (parent.find(mt) != std::string::npos)
          {
            inferred_map = mt;
            break;
          }
        }
      }
      if (!inferred_map.empty())
      {
        map_type_ = inferred_map;
        LOG(INFO) << "Inferred map_type '" << map_type_
                  << "' from traj_file.";
      }
      else
      {
        LOG(ERROR) << "Could not infer map_type from traj_file: " << traj_file_
                   << ". Set ~map_type explicitly.";
        return false;
      }
    }

    if (metric_str_.empty() || metric_str_ == "auto")
    {
      const std::string src = toLower(map_type_);
      if (endsWith(src, "_trace"))
      {
        metric_str_ = "trace";
      }
      else if (endsWith(src, "_det"))
      {
        metric_str_ = "det";
      }
      else
      {
        metric_str_ = "trace";
        LOG(WARNING) << "Could not infer info_metric_type from map_type '"
                     << map_type_ << "', defaulting to trace.";
      }
      LOG(INFO) << "Using info_metric_type '" << metric_str_ << "'.";
    }
    return true;
  }

  bool maybeFallbackPcMap()
  {
    const std::string occ_fn = map_dir_ + "/occ_layer.protobuf";
    const std::string layer_fn = map_dir_ + "/" + map_type_ + "_layer.protobuf";
    if (rpg::fs::fileExists(occ_fn) && rpg::fs::fileExists(layer_fn))
    {
      return true;
    }
    if (map_type_.rfind("pc_", 0) != 0)
    {
      LOG(ERROR) << "Map layers not found in " << map_dir_;
      return false;
    }

    std::string fallback_map;
    if (metric_str_ == "det")
    {
      fallback_map = "gp_info";
    }
    else if (metric_str_ == "trace")
    {
      fallback_map = "gp_trace";
    }
    else
    {
      fallback_map = "gp_info";
    }
    LOG(WARNING) << "Map layers not found in " << map_dir_
                 << ". Falling back to '" << fallback_map << "'.";
    map_type_ = fallback_map;
    if (map_dir_from_root_ || map_dir_.empty())
    {
      map_dir_ = map_root_ + "/" + map_type_ + "_" + map_suffix_;
    }
    return true;
  }

  bool isTraceMap(const std::string& map_type) const
  {
    return map_type.find("trace") != std::string::npos;
  }

  bool computeValues()
  {
    const std::string occ_fn = map_dir_ + "/occ_layer.protobuf";
    const std::string layer_fn = map_dir_ + "/" + map_type_ + "_layer.protobuf";
    if (!rpg::fs::fileExists(occ_fn) || !rpg::fs::fileExists(layer_fn))
    {
      LOG(ERROR) << "Map layers not found in " << map_dir_;
      return false;
    }

    if (map_type_ == "gp_info")
    {
      act_map::GPInfoVoxel::setVisApproxFromFolderGP(gp_vis_dir_);
      act_map::GPTraceVoxel::setVisApproxFromFolderGP(gp_vis_dir_);
      return computeValuesForMap<act_map::GPInfoVoxel>(occ_fn, layer_fn);
    }
    if (map_type_ == "gp_trace")
    {
      act_map::GPInfoVoxel::setVisApproxFromFolderGP(gp_vis_dir_);
      act_map::GPTraceVoxel::setVisApproxFromFolderGP(gp_vis_dir_);
      return computeValuesForMap<act_map::GPTraceVoxel>(occ_fn, layer_fn);
    }
    if (map_type_ == "quad_info")
    {
      return computeValuesForMap<act_map::QuadInfoVoxel>(occ_fn, layer_fn);
    }
    if (map_type_ == "quad_trace")
    {
      return computeValuesForMap<act_map::QuadTraceVoxel>(occ_fn, layer_fn);
    }

    LOG(ERROR) << "Unknown map_type '" << map_type_ << "'";
    return false;
  }

  template <typename VoxelT>
  bool computeValuesForMap(const std::string& occ_fn,
                           const std::string& layer_fn)
  {
    auto map = std::make_shared<act_map::ActMap<VoxelT>>(act_map_opts_);
    map->loadLayers(occ_fn, layer_fn);
    map_vox_size_ = map->kerLayerCRef().voxel_size();

    if (publish_map_heatmap_)
    {
      if (map_heatmap_visible_only_)
      {
        map->cachePointsAndViewDirs(true);
        map_visible_points_ = &(map->cachedPoints());
        map_visible_views_ = &(map->cachedViewDirs());
        if (map_visible_points_->empty())
        {
          LOG(WARNING) << "No cached points for visibility heatmap.";
        }
        else
        {
          map_visible_ready_ = true;
        }

        auto evaluator = std::make_shared<act_map::PosFactorLayerEvaluator<VoxelT>>(
            map->kerLayerPtr().get());
        if (!act_map_opts_.vis_options_.empty())
        {
          const act_map::QuadVisScoreOptions& op = act_map_opts_.vis_options_[0];
          act_map::QuadraticVisScore vis(op.half_fov_rad);
          vis.initSecondOrderApprox(op.boundary_to_mid_ratio, op.boundary_value);
          evaluator->setQuadraticCoefs(vis.k1(), vis.k2(), vis.k3());
        }
        map_voxel_value_fn_ =
            [map, evaluator, this](const Eigen::Vector3d& c,
                                   const Eigen::Matrix3d& Rwc,
                                   double* val) {
              const VoxelT* vox = map->kerLayerCRef().getVoxelPtrByCoordinates(c);
              if (!vox)
              {
                return false;
              }
              (*val) = evaluator->getValueFromVoxel(Rwc, vox, metric_type_, nullptr);
              return std::isfinite(*val);
            };
        map_visible_idx_fn_ =
            [map, this](const rpg::Pose& Twc, act_map::VisIdx* vis_idx) {
              map->visCheckerCRef().getVisibleIdx(
                  Twc, *map_visible_points_, *map_visible_views_, vis_idx);
            };
      }
      else
      {
        collectLayerCenters(map->kerLayerCRef(), map_heatmap_stride_,
                            &map_centers_);
        if (map_heatmap_use_z_slice_)
        {
          std::vector<Eigen::Vector3d> filtered;
          filtered.reserve(map_centers_.size());
          for (const Eigen::Vector3d& c : map_centers_)
          {
            if (std::fabs(c.z() - map_heatmap_z_) <= map_heatmap_z_tol_)
            {
              filtered.emplace_back(c);
            }
          }
          map_centers_.swap(filtered);
        }
        if (map_centers_.empty())
        {
          LOG(WARNING) << "No map centers collected for heatmap.";
        }
      }
    }

    bool pc_ready = false;
    if (use_info_from_pc_)
    {
      act_map::Vec3dVec pts;
      map->getCentersOfOccupiedVoxels(&pts);
      if (!pts.empty())
      {
        map->prepareInfoFromPointCloud();
        pc_ready = true;
      }
      else
      {
        LOG(WARNING) << "No occupied voxels; falling back to map query.";
      }
    }

    values_.resize(traj_.poses.size(), 0.0);
    valid_.resize(traj_.poses.size(), false);
    for (size_t i = 0; i < traj_.poses.size(); i++)
    {
      double val = 0.0;
      bool ok = queryMetric(*map, traj_.poses[i], pc_ready, &val);
      values_[i] = val;
      valid_[i] = ok && std::isfinite(val);
    }

    query_fn_ = [map, this, pc_ready](const rpg::Pose& Twc, double* val) {
      return queryMetric(*map, Twc, pc_ready, val);
    };
    query_ready_ = true;
    return true;
  }

  template <typename VoxelT>
  bool queryMetric(const act_map::ActMap<VoxelT>& map,
                   const rpg::Pose& Twc,
                   const bool pc_ready,
                   double* val) const
  {
    if (use_info_from_pc_ && pc_ready)
    {
      return map.getInfoMetricFromPC(Twc, metric_type_, val, nullptr, nullptr);
    }
    return map.getInfoMetricAt(Twc, metric_type_, val, nullptr, nullptr);
  }

  void computeColorRange()
  {
    if (use_color_bounds_)
    {
      color_min_used_ = color_min_;
      color_max_used_ = color_max_;
      if (color_max_used_ <= color_min_used_)
      {
        color_max_used_ = color_min_used_ + 1.0;
      }
      return;
    }

    double vmin = std::numeric_limits<double>::infinity();
    double vmax = -std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < values_.size(); i++)
    {
      if (!valid_[i])
      {
        continue;
      }
      vmin = std::min(vmin, values_[i]);
      vmax = std::max(vmax, values_[i]);
    }
    if (!std::isfinite(vmin) || !std::isfinite(vmax))
    {
      vmin = 0.0;
      vmax = 1.0;
    }
    if (vmax <= vmin)
    {
      vmax = vmin + 1.0;
    }
    color_min_used_ = vmin;
    color_max_used_ = vmax;
  }

  std_msgs::ColorRGBA colorForValue(const double v, const bool valid) const
  {
    if (!valid || !std::isfinite(v))
    {
      return grayColor(heatmap_alpha_);
    }
    const double t = (v - color_min_used_) / (color_max_used_ - color_min_used_);
    return jetColor(t, heatmap_alpha_);
  }

  void publishPath()
  {
    std_msgs::ColorRGBA start_c;
    start_c.r = 0.0;
    start_c.g = 1.0;
    start_c.b = 0.0;
    start_c.a = 1.0;
    std_msgs::ColorRGBA end_c;
    end_c.r = 1.0;
    end_c.g = 0.0;
    end_c.b = 0.0;
    end_c.a = 1.0;
    act_map_exp::visualizePath(traj_.poses, start_c, end_c, path_pub_, 0,
                               frame_id_, "rrt_path");
  }

  void publishHeatmap(const size_t upto)
  {
    if (!publish_heatmap_)
    {
      return;
    }
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = "rrt_heatmap";
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = heatmap_scale_;
    m.scale.y = heatmap_scale_;
    m.scale.z = heatmap_scale_;

    const size_t end =
        std::min(upto + 1, static_cast<size_t>(traj_.poses.size()));
    const size_t stride = std::max(1, heatmap_stride_);
    m.points.reserve((end + stride - 1) / stride);
    m.colors.reserve((end + stride - 1) / stride);
    for (size_t i = 0; i < end; i += stride)
    {
      geometry_msgs::Point p;
      p.x = traj_.poses[i].getPosition().x();
      p.y = traj_.poses[i].getPosition().y();
      p.z = traj_.poses[i].getPosition().z();
      m.points.push_back(p);
      m.colors.push_back(colorForValue(values_[i], valid_[i]));
    }
    heatmap_pub_.publish(m);
  }

  void publishMapHeatmap(const rpg::Pose& Twc)
  {
    if (!publish_map_heatmap_ || !query_ready_)
    {
      return;
    }
    const Eigen::Matrix3d Rwc = Twc.getRotationMatrix();
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = "rrt_map_heatmap";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    const double vox_scale = map_vox_size_ * map_heatmap_vox_scale_;
    m.scale.x = vox_scale;
    m.scale.y = vox_scale;
    m.scale.z = vox_scale;

    if (map_heatmap_visible_only_)
    {
      if (!map_visible_ready_ || !map_visible_points_ || !map_visible_views_ ||
          !map_voxel_value_fn_ || !map_visible_idx_fn_)
      {
        LOG(WARNING) << "Visible heatmap requested but visibility cache not ready.";
        return;
      }
      act_map::VisIdx vis_idx;
      map_visible_idx_fn_(Twc, &vis_idx);
      if (vis_idx.empty())
      {
        LOG(WARNING) << "No visible voxels for current pose.";
        return;
      }
      const size_t stride = std::max(1, map_heatmap_stride_);
      m.points.reserve(vis_idx.size());
      m.colors.reserve(vis_idx.size());
      size_t k = 0;
      for (const size_t idx : vis_idx)
      {
        if ((k++ % stride) != 0)
        {
          continue;
        }
        const Eigen::Vector3d& c = (*map_visible_points_)[idx];
        if (map_heatmap_use_z_slice_ &&
            std::fabs(c.z() - map_heatmap_z_) > map_heatmap_z_tol_)
        {
          continue;
        }
        double val = 0.0;
        bool ok = map_voxel_value_fn_(c, Rwc, &val);
        geometry_msgs::Point p;
        p.x = c.x();
        p.y = c.y();
        p.z = c.z();
        m.points.push_back(p);
        std_msgs::ColorRGBA col = colorForValue(val, ok && std::isfinite(val));
        col.a = map_heatmap_alpha_;
        m.colors.push_back(col);
      }
    }
    else
    {
      if (!query_ready_ || map_centers_.empty())
      {
        return;
      }
      const rpg::Rotation rot(Rwc);
      m.points.reserve(map_centers_.size());
      m.colors.reserve(map_centers_.size());
      for (const Eigen::Vector3d& c : map_centers_)
      {
        rpg::Pose Twc_c(c, rot);
        double val = 0.0;
        bool ok = query_fn_(Twc_c, &val);
        geometry_msgs::Point p;
        p.x = c.x();
        p.y = c.y();
        p.z = c.z();
        m.points.push_back(p);
        std_msgs::ColorRGBA col = colorForValue(val, ok && std::isfinite(val));
        col.a = map_heatmap_alpha_;
        m.colors.push_back(col);
      }
    }
    map_heatmap_pub_.publish(m);
  }

  void publishCamera(const size_t idx)
  {
    if (!publish_camera_ || idx >= traj_.poses.size())
    {
      return;
    }
    const Eigen::Vector3d cam_color(0.0, 0.8, 1.0);
    act_map_ros::publishCameraMarker(cam_pub_, frame_id_, "rrt_cam",
                                     ros::Time::now(), 0,
                                     visualization_msgs::Marker::ADD,
                                     camera_marker_scale_, cam_color,
                                     traj_.poses[idx]);
  }

  void publishValueText(const size_t idx)
  {
    if (!publish_value_text_ || idx >= traj_.poses.size())
    {
      return;
    }
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = "rrt_value";
    m.id = 0;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = traj_.poses[idx].getPosition().x();
    m.pose.position.y = traj_.poses[idx].getPosition().y();
    m.pose.position.z = traj_.poses[idx].getPosition().z() + text_z_offset_;
    m.scale.z = text_scale_;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);
    ss << values_[idx];
    m.text = ss.str();
    text_pub_.publish(m);
  }

  void publishCurrent()
  {
    if (traj_.poses.empty())
    {
      return;
    }
    publishCamera(cur_idx_);
    if (publish_value_text_)
    {
      publishValueText(cur_idx_);
    }
    if (publish_heatmap_ && heatmap_mode_ == "visited")
    {
      publishHeatmap(cur_idx_);
    }
    if (publish_map_heatmap_ && map_heatmap_mode_ == "follow")
    {
      publishMapHeatmap(traj_.poses[cur_idx_]);
    }
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (paused_)
    {
      return;
    }
    if (traj_.poses.empty())
    {
      return;
    }
    if (publish_map_heatmap_ && map_heatmap_mode_ == "follow" &&
        map_heatmap_update_every_n_ > 1 &&
        (cur_idx_ % static_cast<size_t>(map_heatmap_update_every_n_) != 0))
    {
      // temporarily disable map update for this step
      const bool keep_map = publish_map_heatmap_;
      publish_map_heatmap_ = false;
      publishCurrent();
      publish_map_heatmap_ = keep_map;
    }
    else
    {
      publishCurrent();
    }

    if (cur_idx_ + 1 < traj_.poses.size())
    {
      cur_idx_++;
      return;
    }
    if (loop_)
    {
      cur_idx_ = 0;
    }
    else if (timer_.hasStarted())
    {
      timer_.stop();
    }
  }

  bool nextPoseCallback(std_srvs::Empty::Request&,
                        std_srvs::Empty::Response&)
  {
    if (traj_.poses.empty())
    {
      return true;
    }
    if (cur_idx_ + 1 < traj_.poses.size())
    {
      cur_idx_++;
    }
    else if (loop_)
    {
      cur_idx_ = 0;
    }
    publishCurrent();
    return true;
  }

  bool prevPoseCallback(std_srvs::Empty::Request&,
                        std_srvs::Empty::Response&)
  {
    if (traj_.poses.empty())
    {
      return true;
    }
    if (cur_idx_ > 0)
    {
      cur_idx_--;
    }
    else if (loop_)
    {
      cur_idx_ = traj_.poses.size() - 1;
    }
    publishCurrent();
    return true;
  }

  bool resetPoseCallback(std_srvs::Empty::Request&,
                         std_srvs::Empty::Response&)
  {
    if (traj_.poses.empty())
    {
      return true;
    }
    cur_idx_ = 0;
    publishCurrent();
    return true;
  }

  bool setPausedCallback(std_srvs::SetBool::Request& req,
                         std_srvs::SetBool::Response& res)
  {
    paused_ = req.data;
    res.success = true;
    res.message = paused_ ? "paused" : "running";
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string traj_file_;
  std::string traj_format_str_;
  std::string map_type_;
  std::string metric_str_;
  std::string act_map_params_;
  std::string gp_vis_dir_;
  std::string map_root_;
  std::string map_suffix_;
  std::string map_dir_;
  bool map_dir_from_root_ = false;
  std::string frame_id_;

  double playback_hz_ = 2.0;
  bool loop_ = true;
  bool publish_heatmap_ = true;
  bool publish_path_ = true;
  bool publish_camera_ = true;
  bool publish_value_text_ = false;
  bool use_info_from_pc_ = false;

  std::string heatmap_mode_;
  double heatmap_scale_ = 0.2;
  double heatmap_alpha_ = 0.9;
  int heatmap_stride_ = 1;
  bool use_color_bounds_ = false;
  double color_min_ = 0.0;
  double color_max_ = 1.0;
  double color_min_used_ = 0.0;
  double color_max_used_ = 1.0;

  double text_scale_ = 0.6;
  double text_z_offset_ = 0.5;
  double camera_marker_scale_ = 0.5;

  bool initialized_ = false;
  size_t cur_idx_ = 0;
  bool manual_mode_ = false;
  bool paused_ = false;

  Trajectory traj_;
  std::vector<double> values_;
  std::vector<bool> valid_;

  act_map::InfoMetricType metric_type_ = act_map::InfoMetricType::kTrace;
  act_map::ActMapOptions act_map_opts_;
  double map_vox_size_ = 1.0;
  std::vector<Eigen::Vector3d> map_centers_;
  bool query_ready_ = false;
  std::function<bool(const rpg::Pose&, double*)> query_fn_;

  bool publish_map_heatmap_ = false;
  std::string map_heatmap_mode_;
  int map_heatmap_stride_ = 3;
  double map_heatmap_alpha_ = 0.8;
  double map_heatmap_vox_scale_ = 0.9;
  int map_heatmap_update_every_n_ = 5;
  bool map_heatmap_use_z_slice_ = false;
  double map_heatmap_z_ = 2.0;
  double map_heatmap_z_tol_ = 0.25;
  bool map_heatmap_visible_only_ = false;
  bool map_visible_ready_ = false;
  const act_map::Vec3dVec* map_visible_points_ = nullptr;
  const act_map::Vec3dVec* map_visible_views_ = nullptr;
  std::function<bool(const Eigen::Vector3d&, const Eigen::Matrix3d&, double*)>
      map_voxel_value_fn_;
  std::function<void(const rpg::Pose&, act_map::VisIdx*)> map_visible_idx_fn_;

  ros::Publisher heatmap_pub_;
  ros::Publisher map_heatmap_pub_;
  ros::Publisher path_pub_;
  ros::Publisher cam_pub_;
  ros::Publisher text_pub_;
  ros::Timer timer_;

  ros::ServiceServer next_pose_srv_;
  ros::ServiceServer prev_pose_srv_;
  ros::ServiceServer reset_pose_srv_;
  ros::ServiceServer set_paused_srv_;
};

RPG_COMMON_MAIN
{
  ros::init(argc, argv, "rrt_info_viz");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  RRTInfoViz node(nh, pnh);
  if (!node.ok())
  {
    LOG(ERROR) << "Failed to initialize rrt_info_viz node.";
    return 1;
  }

  ros::spin();
  return 0;
}
