#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cctype>
#include <vector>
#include <map>

#include "act_map/voxblox/io/layer_io.h"
#include "act_map/voxblox/core/layer.h"
#include "act_map/voxblox/core/voxel.h"
#include "act_map/act_map.h"
#include "act_map/information_potential.h"
#include "act_map/info_utils.h"
#include "act_map/common.h"
#include "act_map_exp/exp_utils.h"
#include "rpg_common/pose.h"

using act_map::InfoMetricType;
using act_map::voxblox::EsdfVoxel;

struct Options {
  std::string rrt_dir;
  std::string config_yaml;
  std::string variation_dir;
  std::string esdf_vxblx;
  std::string act_map_params;
  std::string info_map_dir;
  std::string info_map_root;
  std::string info_map_suffix;
  std::string info_map_suffixes;
  std::string map_type;  // gp_info, gp_trace, quad_info, quad_trace
  std::string gp_vis_dir;
  double robot_radius = -1.0;
  bool verbose = false;
  bool only_invalid = true;
};

struct CheckResult {
  bool ok = false;
  double value = 0.0;
  double thresh = 0.0;
};

struct BaseCfg {
  std::string path;
  std::string name;
  YAML::Node node;
};

struct VarCfg {
  std::string path;
  std::string type;
  std::string node_name;
  std::string map_key;
  YAML::Node node;
};

struct MapSpec {
  std::string name;
  std::string layer;
  std::string subdir_prefix;
  InfoMetricType default_metric;
  bool force_trace;
  bool is_gp;
};

std::string repoRoot() {
  char cwd[4096];
  if (!getcwd(cwd, sizeof(cwd))) return "";
  return std::string(cwd);
}

bool fileExists(const std::string& path) {
  struct stat st;
  return stat(path.c_str(), &st) == 0;
}

std::vector<std::string> listYamlFiles(const std::string& dir) {
  std::vector<std::string> out;
  DIR* dp = opendir(dir.c_str());
  if (!dp) return out;
  struct dirent* ep;
  while ((ep = readdir(dp)) != nullptr) {
    std::string name = ep->d_name;
    if (name == "." || name == "..") continue;
    if (name.size() >= 5 && name.substr(name.size() - 5) == ".yaml") {
      out.emplace_back(dir + "/" + name);
    }
  }
  closedir(dp);
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<std::string> splitCommaList(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else if (!std::isspace(static_cast<unsigned char>(c))) {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

std::string dirName(const std::string& path) {
  size_t pos = path.find_last_of("/\\");
  if (pos == std::string::npos) return ".";
  return path.substr(0, pos);
}

void parseArgs(int argc, char** argv, Options* opt) {
  std::map<std::string, std::string> kv;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string k = argv[i];
    std::string v = argv[i + 1];
    kv[k] = v;
  }
  if (kv.count("--rrt-dir")) opt->rrt_dir = kv["--rrt-dir"];
  if (kv.count("--config-yaml")) opt->config_yaml = kv["--config-yaml"];
  if (kv.count("--variation-dir")) opt->variation_dir = kv["--variation-dir"];
  if (kv.count("--esdf")) opt->esdf_vxblx = kv["--esdf"];
  if (kv.count("--act-map-params")) opt->act_map_params = kv["--act-map-params"];
  if (kv.count("--info-map-dir")) opt->info_map_dir = kv["--info-map-dir"];
  if (kv.count("--info-map-root")) opt->info_map_root = kv["--info-map-root"];
  if (kv.count("--info-map-suffix")) opt->info_map_suffix = kv["--info-map-suffix"];
  if (kv.count("--info-map-suffixes")) opt->info_map_suffixes = kv["--info-map-suffixes"];
  if (kv.count("--map-type")) opt->map_type = kv["--map-type"];
  if (kv.count("--gp-vis-dir")) opt->gp_vis_dir = kv["--gp-vis-dir"];
  if (kv.count("--robot-radius")) opt->robot_radius = std::stod(kv["--robot-radius"]);
  if (kv.count("--verbose")) opt->verbose = (kv["--verbose"] == "1" || kv["--verbose"] == "true");
  if (kv.count("--only-invalid")) opt->only_invalid = (kv["--only-invalid"] == "1" || kv["--only-invalid"] == "true");
}

act_map::ActMapOptions readActMapOptions(const std::string& fn) {
  YAML::Node node = YAML::LoadFile(fn);
  act_map::ActMapOptions opts;
  opts.occ_layer_options_.vox_size = node["occ_vox_size"].as<double>();
  opts.occ_layer_options_.vox_per_side = node["occ_vox_per_side"].as<int>();
  opts.pos_factor_layer_options_.vox_size = node["ker_vox_size"].as<double>();
  opts.pos_factor_layer_options_.vox_per_side = node["ker_vox_per_side"].as<int>();

  opts.occ_integrator_options_.probability_hit = node["occ_prob_hit"].as<double>();
  opts.occ_integrator_options_.probability_miss = node["occ_prob_miss"].as<double>();
  opts.occ_integrator_options_.threshold_min = node["occ_thresh_min"].as<double>();
  opts.occ_integrator_options_.threshold_max = node["occ_thresh_max"].as<double>();
  opts.occ_integrator_options_.threshold_occupancy = node["occ_thresh_occupancy"].as<double>();
  opts.occ_integrator_options_.min_ray_length_m = node["occ_min_ray_m"].as<double>();
  opts.occ_integrator_options_.max_ray_length_m = node["occ_max_ray_m"].as<double>();

  act_map::QuadVisScoreOptions vis;
  vis.half_fov_rad = node["vis_half_fov_rad0"].as<double>();
  vis.boundary_to_mid_ratio = node["vis_boundary_ratio0"].as<double>();
  vis.boundary_value = node["vis_boundary_val0"].as<double>();
  opts.vis_options_.push_back(vis);

  opts.pos_fac_integrator_options_.occ_thresh_ = node["ker_inte_occ_thresh"].as<double>();

  opts.vis_checker_options_.min_dist = node["vis_check_min_dist"].as<double>();
  opts.vis_checker_options_.max_dist = node["vis_check_max_dist"].as<double>();
  opts.vis_checker_options_.use_view_filtering = node["vis_check_use_view_filter"].as<bool>();
  double max_ang_deg = node["vis_check_max_ang_deg"].as<double>();
  opts.vis_checker_options_.min_view_angle_cos = std::cos(max_ang_deg * M_PI / 180.0);
  opts.vis_checker_options_.use_depth_layer_ = node["vis_check_use_depth_layer"].as<bool>();
  opts.vis_checker_options_.dm_options_.depth_layer_opts_.vox_size =
      node["depth_layer_vox_size"].as<double>();
  opts.vis_checker_options_.dm_options_.depth_layer_opts_.vox_per_side =
      node["depth_layer_vox_per_side"].as<int>();
  opts.vis_checker_options_.dm_options_.depth_voxel_step_deg_ =
      node["depth_vox_deg_step"].as<double>();

  opts.use_collision_checker_ = node["use_collision_checker"].as<bool>();
  opts.col_ops_.min_dist_thresh_ = node["occ_min_dist_thresh"].as<double>();
  opts.col_ops_.average_dist_thresh = node["occ_average_dist_thresh"].as<double>();

  opts.eval_method = act_map::EvaluateStrategy::kInterpolation;
  return opts;
}

act_map::InfoPotentialOptions readInfoPotOptions(const YAML::Node& node) {
  act_map::InfoPotentialOptions opts;
  if (node["info_pot_min_depth_m"]) {
    opts.min_depth_m_ = node["info_pot_min_depth_m"].as<double>();
  }
  if (node["info_pot_max_depth_m"]) {
    opts.max_depth_m_ = node["info_pot_max_depth_m"].as<double>();
  }
  if (node["info_pot_n_random_lm"]) {
    opts.n_random_landmarks_ = node["info_pot_n_random_lm"].as<int>();
  }
  return opts;
}

Eigen::Matrix4d readTbc(const YAML::Node& node) {
  Eigen::Matrix4d Tbc = Eigen::Matrix4d::Identity();
  if (!node["T_BC"]) return Tbc;
  const YAML::Node& T = node["T_BC"];
  if (!T.IsSequence() || T.size() < 16) return Tbc;
  for (size_t i = 0; i < 16; ++i) {
    Tbc(i / 4, i % 4) = T[i].as<double>();
  }
  return Tbc;
}

bool getBool(const YAML::Node& node, const std::string& key, bool def_val) {
  if (node[key]) {
    return node[key].as<bool>();
  }
  return def_val;
}

YAML::Node mergeNodes(const YAML::Node& base, const YAML::Node& var) {
  YAML::Node out = YAML::Clone(base);
  for (auto it = var.begin(); it != var.end(); ++it) {
    out[it->first] = it->second;
  }
  return out;
}

std::string mapKeyFromNode(const std::string& node_name) {
  if (node_name.find("gp_trace") != std::string::npos) return "gp_trace";
  if (node_name.find("gp_info") != std::string::npos) return "gp_info";
  if (node_name.find("quadratic_trace") != std::string::npos) return "quad_trace";
  if (node_name.find("quadratic_info") != std::string::npos) return "quad_info";
  return "";
}

std::vector<BaseCfg> loadBasesFromConfig(const std::string& cfg_fn) {
  std::vector<BaseCfg> bases;
  if (!fileExists(cfg_fn)) return bases;
  YAML::Node cfg = YAML::LoadFile(cfg_fn);
  const std::string cfg_dir = dirName(cfg_fn);
  for (auto it = cfg.begin(); it != cfg.end(); ++it) {
    const YAML::Node group = it->second;
    if (!group["base"]) continue;
    const YAML::Node base = group["base"];
    for (auto bit = base.begin(); bit != base.end(); ++bit) {
      std::string fn = bit->first.as<std::string>();
      std::string name = bit->second.as<std::string>();
      if (!fn.empty() && fn[0] != '/') fn = cfg_dir + "/" + fn;
      if (!fileExists(fn)) continue;
      BaseCfg b;
      b.path = fn;
      b.name = name;
      b.node = YAML::LoadFile(fn);
      bases.push_back(b);
    }
  }
  return bases;
}

std::vector<std::string> loadVariationsFromConfig(const std::string& cfg_fn) {
  std::vector<std::string> vars;
  if (!fileExists(cfg_fn)) return vars;
  YAML::Node cfg = YAML::LoadFile(cfg_fn);
  const std::string cfg_dir = dirName(cfg_fn);
  for (auto it = cfg.begin(); it != cfg.end(); ++it) {
    const YAML::Node group = it->second;
    if (!group["var"]) continue;
    const YAML::Node var = group["var"];
    for (size_t i = 0; i < var.size(); ++i) {
      std::string fn = var[i].as<std::string>();
      if (!fn.empty() && fn[0] != '/') fn = cfg_dir + "/" + fn;
      if (fileExists(fn)) vars.push_back(fn);
    }
  }
  std::sort(vars.begin(), vars.end());
  vars.erase(std::unique(vars.begin(), vars.end()), vars.end());
  return vars;
}

bool queryEsdf(const act_map::voxblox::Layer<EsdfVoxel>::Ptr& esdf,
               const Eigen::Vector3d& pos, double* dist) {
  if (!esdf) return false;
  const EsdfVoxel* v = esdf->getVoxelPtrByCoordinates(pos);
  if (!v) return false;
  if (!v->observed) return false;
  *dist = v->distance;
  return true;
}

template <typename VoxelT>
std::shared_ptr<act_map::ActMap<VoxelT>> loadActMap(
    const act_map::ActMapOptions& opts,
    const std::string& dir,
    const std::string& layer_name) {
  auto map = std::make_shared<act_map::ActMap<VoxelT>>(opts);
  std::string occ = dir + "/occ_layer.protobuf";
  std::string layer = dir + "/" + layer_name + "_layer.protobuf";
  map->loadLayers(occ, layer);
  return map;
}

InfoMetricType parseMetric(const YAML::Node& node, const std::string& default_name) {
  if (node["info_metric_type"]) {
    const std::string m = node["info_metric_type"].as<std::string>();
    if (act_map::kNameToInfoMetric.count(m)) {
      return act_map::kNameToInfoMetric[m];
    }
  }
  if (act_map::kNameToInfoMetric.count(default_name)) {
    return act_map::kNameToInfoMetric[default_name];
  }
  return InfoMetricType::kDet;
}

template <typename VoxelT>
CheckResult checkInfoAt(const act_map::ActMap<VoxelT>& map,
                        const act_map::InformationPotential<VoxelT>& pot,
                        const InfoMetricType metric,
                        const rpg::Pose& Twc,
                        const bool use_pc);

struct PoseReport {
  bool esdf_ok = true;
  double esdf_dist = 0.0;
  bool info_ok = true;
  double info_val = 0.0;
  double info_thresh = 0.0;
  bool ok = true;
  bool used_esdf = false;
  bool used_info = false;
  std::string note;
};

template <typename VoxelT>
PoseReport checkPose(const Eigen::Vector3d& pos, double yaw_deg,
                     const bool use_joint, const bool use_esdf, const bool use_pc,
                     const bool pc_ready,
                     const act_map::voxblox::Layer<EsdfVoxel>::Ptr& esdf_layer,
                     const double robot_radius,
                     const act_map::ActMap<VoxelT>& map,
                     const act_map::InformationPotential<VoxelT>& pot,
                     const InfoMetricType metric,
                     const rpg::Pose& Tbc) {
  PoseReport rep;
  rep.used_esdf = use_joint || use_esdf;
  rep.used_info = use_joint;
  if (rep.used_esdf) {
    double dist = 0.0;
    rep.esdf_ok = queryEsdf(esdf_layer, pos, &dist) && (dist > robot_radius);
    rep.esdf_dist = dist;
    if (!rep.esdf_ok) {
      rep.ok = false;
      return rep;
    }
  }
  if (use_joint) {
    Eigen::Matrix3d R;
    act_map_exp::quadAccYawToRwb(Eigen::Vector3d::Zero(),
                                 yaw_deg * M_PI / 180.0,
                                 &R, nullptr, nullptr);
    rpg::Pose Twb(rpg::Rotation(R), pos);
    rpg::Pose Twc = Twb * Tbc;
    if (use_pc && !pc_ready) {
      rep.info_ok = false;
      rep.info_val = 0.0;
      rep.info_thresh = pot.getMetricThresh(metric);
      rep.ok = false;
      rep.note = "pc_cache_empty";
      return rep;
    }
    CheckResult info = checkInfoAt(map, pot, metric, Twc, use_pc);
    rep.info_ok = info.ok;
    rep.info_val = info.value;
    rep.info_thresh = info.thresh;
    if (!rep.info_ok) {
      rep.ok = false;
      return rep;
    }
  }
  rep.ok = true;
  return rep;
}

template <typename VoxelT>
CheckResult checkInfoAt(const act_map::ActMap<VoxelT>& map,
                        const act_map::InformationPotential<VoxelT>& pot,
                        const InfoMetricType metric,
                        const rpg::Pose& Twc,
                        const bool use_pc) {
  CheckResult res;
  double val = 0.0;
  bool ok = false;
  if (use_pc) {
    ok = map.getInfoMetricFromPC(Twc, metric, &val, nullptr, nullptr);
    res.thresh = pot.getMetricThresh(metric);
  } else {
    ok = map.getInfoMetricAt(Twc, metric, &val, nullptr, nullptr);
    res.thresh = pot.getMetricThreshApproxVis(metric);
  }
  if (!ok) {
    val = 1e-6;
  }
  res.ok = ok && (val >= res.thresh);
  res.value = val;
  return res;
}

template <typename VoxelT>
void runSpecForMap(const MapSpec& spec,
                   const std::string& map_label,
                   const std::string& map_dir,
                   const std::vector<BaseCfg>& bases,
                   const std::vector<VarCfg>& vars,
                   const act_map::ActMapOptions& am_opts,
                   const act_map::voxblox::Layer<EsdfVoxel>::Ptr& esdf_layer,
                   const double robot_radius,
                   const bool verbose,
                   const bool only_invalid) {
  if (!fileExists(map_dir + "/occ_layer.protobuf")) {
    std::cerr << "info map dir missing layers: " << map_dir << std::endl;
    return;
  }

  std::vector<const VarCfg*> var_list;
  for (const auto& v : vars) {
    if (v.map_key == spec.name) {
      var_list.push_back(&v);
    }
  }
  if (var_list.empty()) {
    return;
  }

  auto map = loadActMap<VoxelT>(am_opts, map_dir, spec.layer);

  bool need_pc = false;
  for (const auto& v : var_list) {
    if (getBool(v->node, "calculate_info_from_pc", false)) {
      need_pc = true;
      break;
    }
  }
  if (!need_pc) {
    for (const auto& b : bases) {
      if (getBool(b.node, "calculate_info_from_pc", false)) {
        need_pc = true;
        break;
      }
    }
  }

  bool pc_ready = false;
  if (need_pc) {
    act_map::Vec3dVec pts;
    map->getCentersOfOccupiedVoxels(&pts);
    if (!pts.empty()) {
      map->prepareInfoFromPointCloud();
      pc_ready = true;
    } else {
      pc_ready = false;
    }
  }

  std::cout << "=== map_suffix " << map_label << " (" << spec.name << ") ===" << std::endl;

  auto print_pose = [&](const PoseReport& r, const std::string& label) {
    std::cout << "  " << label << ": " << (r.ok ? "OK" : "FAIL");
    if (r.used_esdf) {
      std::cout << " ESDF=" << r.esdf_dist;
    }
    if (r.used_info) {
      std::cout << " INFO=" << r.info_val << " (thr=" << r.info_thresh << ")";
    }
    if (!r.note.empty()) {
      std::cout << " NOTE=" << r.note;
    }
    std::cout << "\n";
  };

  for (const auto& b : bases) {
    if (!b.node["start"] || !b.node["end"]) continue;
    for (const auto* v : var_list) {
      YAML::Node merged = mergeNodes(b.node, v->node);
      if (!merged["start"] || !merged["end"]) continue;
      Eigen::Vector3d s(merged["start"][0].as<double>(),
                        merged["start"][1].as<double>(),
                        merged["start"][2].as<double>());
      Eigen::Vector3d e(merged["end"][0].as<double>(),
                        merged["end"][1].as<double>(),
                        merged["end"][2].as<double>());
      double syaw = merged["start_yaw_deg"].as<double>();
      double eyaw = merged["end_yaw_deg"].as<double>();
      const bool use_joint = getBool(merged, "use_joint_checker", false);
      const bool use_esdf = getBool(merged, "use_esdf_checker", false);
      const bool use_pc = getBool(merged, "calculate_info_from_pc", false);
      act_map::InformationPotential<VoxelT> pot(readInfoPotOptions(merged),
                                                am_opts.vis_options_[0]);
      InfoMetricType metric = spec.force_trace ? InfoMetricType::kTrace
                                               : parseMetric(merged, "det");
      Eigen::Matrix4d Tbc_mat = readTbc(merged);
      Eigen::Matrix3d Rbc = Tbc_mat.block<3,3>(0,0);
      Eigen::Vector3d tbc = Tbc_mat.block<3,1>(0,3);
      rpg::Pose Tbc(rpg::Rotation(Rbc), tbc);

      PoseReport rs = checkPose(s, syaw, use_joint, use_esdf, use_pc, pc_ready,
                                esdf_layer, robot_radius, *map, pot, metric, Tbc);
      PoseReport re = checkPose(e, eyaw, use_joint, use_esdf, use_pc, pc_ready,
                                esdf_layer, robot_radius, *map, pot, metric, Tbc);
      bool print_entry = false;
      if (only_invalid) {
        print_entry = (!rs.ok || !re.ok);
      } else {
        print_entry = (verbose || !rs.ok || !re.ok);
      }
      if (print_entry) {
        std::cout << b.name << " + " << v->type << " (" << v->node_name << ")\n";
        print_pose(rs, "start");
        print_pose(re, "end");
        std::cout << " => " << ((rs.ok && re.ok) ? "VALID" : "INVALID") << "\n";
      }
    }
  }
}

double readRobotRadius(const std::string& base_yaml) {
  if (!fileExists(base_yaml)) return 2.0;
  YAML::Node node = YAML::LoadFile(base_yaml);
  if (node["robot_radius"]) {
    return node["robot_radius"].as<double>();
  }
  return 2.0;
}

int main(int argc, char** argv) {
  Options opt;
  parseArgs(argc, argv, &opt);

  const std::string root = repoRoot();
  if (opt.rrt_dir.empty())
    opt.rrt_dir = root + "/act_map_exp/params/quad_rrt/warehouse";
  if (opt.esdf_vxblx.empty())
    opt.esdf_vxblx = root + "/act_map_exp/exp_data/warehouse_voxblox/tsdf_esdf_max10.vxblx";
  if (opt.act_map_params.empty())
    opt.act_map_params = root + "/act_map_ros/params/act_map_warehouse.yaml";
  if (opt.info_map_root.empty())
    opt.info_map_root = root + "/act_map_exp/exp_data/warehouse_FIF";
  if (opt.info_map_suffix.empty())
    opt.info_map_suffix = "r2_a20";
  if (opt.gp_vis_dir.empty())
    opt.gp_vis_dir = root + "/act_map/params/fov_approximator_gp/fov45_fs70_lm1000_k15";

  if (opt.robot_radius <= 0.0) {
    opt.robot_radius = readRobotRadius(root + "/act_map_exp/params/quad_rrt/quad_rrt_warehouse_base.yaml");
  }

  if (!fileExists(opt.esdf_vxblx)) {
    std::cerr << "ESDF file missing: " << opt.esdf_vxblx << std::endl;
    return 1;
  }
  if (!fileExists(opt.act_map_params)) {
    std::cerr << "act_map params missing: " << opt.act_map_params << std::endl;
    return 1;
  }
  // Load ESDF layer
  std::cerr << "Loading ESDF layer..." << std::endl;
  act_map::voxblox::Layer<EsdfVoxel>::Ptr esdf_layer;
  if (!act_map::voxblox::io::LoadLayer<EsdfVoxel>(opt.esdf_vxblx, true, &esdf_layer)) {
    std::cerr << "Failed to load ESDF layer from " << opt.esdf_vxblx << std::endl;
    return 1;
  }

  act_map::ActMapOptions am_opts = readActMapOptions(opt.act_map_params);
  act_map::kInfoMetricUseLogDet = true;

  std::vector<BaseCfg> bases;
  std::vector<std::string> var_paths;
  if (!opt.config_yaml.empty()) {
    bases = loadBasesFromConfig(opt.config_yaml);
    var_paths = loadVariationsFromConfig(opt.config_yaml);
  }

  const std::vector<std::string> exclude = {
    "warehouse_all.yaml", "warehouse_mini.yaml", "warehouse_rrt_trial.yaml"
  };

  if (bases.empty()) {
    std::vector<std::string> yamls = listYamlFiles(opt.rrt_dir);
    for (const auto& y : yamls) {
      std::string base = y.substr(y.find_last_of('/') + 1);
      if (std::find(exclude.begin(), exclude.end(), base) != exclude.end()) continue;
      BaseCfg b;
      b.path = y;
      b.name = base;
      if (b.name.size() > 5 && b.name.substr(b.name.size() - 5) == ".yaml") {
        b.name = b.name.substr(0, b.name.size() - 5);
      }
      b.node = YAML::LoadFile(y);
      bases.push_back(b);
    }
  }

  if (opt.variation_dir.empty()) {
    opt.variation_dir = opt.rrt_dir + "/variations";
  }
  if (var_paths.empty() && fileExists(opt.variation_dir)) {
    var_paths = listYamlFiles(opt.variation_dir);
  }

  std::vector<VarCfg> vars;
  for (const auto& vp : var_paths) {
    if (!fileExists(vp)) continue;
    YAML::Node v = YAML::LoadFile(vp);
    if (!v["node"] || !v["type"]) continue;
    VarCfg vc;
    vc.path = vp;
    vc.type = v["type"].as<std::string>();
    vc.node_name = v["node"].as<std::string>();
    vc.map_key = mapKeyFromNode(vc.node_name);
    vc.node = v;
    if (vc.map_key.empty()) continue;
    vars.push_back(vc);
  }

  if (!opt.only_invalid || opt.verbose) {
    std::cout << "Loaded variations (" << vars.size() << "): ";
    for (size_t i = 0; i < vars.size(); ++i) {
      if (i) std::cout << ", ";
      std::cout << vars[i].type << "@" << vars[i].node_name;
    }
    std::cout << std::endl;
  }

  if (bases.empty()) {
    std::cerr << "No base yaml files found." << std::endl;
    return 1;
  }
  if (vars.empty()) {
    std::cerr << "No variation yaml files found." << std::endl;
    return 1;
  }

  std::vector<std::string> suffixes;
  if (!opt.info_map_suffixes.empty()) {
    suffixes = splitCommaList(opt.info_map_suffixes);
  } else if (!opt.info_map_suffix.empty()) {
    suffixes = splitCommaList(opt.info_map_suffix);
  }
  if (suffixes.empty()) {
    suffixes.push_back("r2_a20");
  }

  std::vector<MapSpec> specs = {
      {"gp_info", "gp_info", "gp_info", InfoMetricType::kDet, false, true},
      {"gp_trace", "gp_trace", "gp_trace", InfoMetricType::kTrace, true, true},
      {"quad_info", "quad_info", "quad_info", InfoMetricType::kDet, false, false},
      {"quad_trace", "quad_trace", "quad_trace", InfoMetricType::kTrace, true, false},
  };

  if (opt.info_map_dir.empty()) {
    for (const auto& suf : suffixes) {
      for (const auto& s : specs) {
        if (!opt.map_type.empty() && s.name != opt.map_type) continue;
        std::string map_dir = opt.info_map_root + "/" + s.subdir_prefix + "_" + suf;
        if (s.is_gp) {
          act_map::GPInfoVoxel::setVisApproxFromFolderGP(opt.gp_vis_dir);
          act_map::GPTraceVoxel::setVisApproxFromFolderGP(opt.gp_vis_dir);
        } else {
          act_map::QuadPolyInfoVoxel::setVisApproxQuadVisOpt(am_opts.vis_options_[0]);
          act_map::QuadPolyTraceVoxel::setVisApproxQuadVisOpt(am_opts.vis_options_[0]);
        }
        if (s.name == "gp_info") {
          runSpecForMap<act_map::GPInfoVoxel>(s, suf, map_dir, bases, vars,
                                              am_opts, esdf_layer, opt.robot_radius,
                                              opt.verbose, opt.only_invalid);
        } else if (s.name == "gp_trace") {
          runSpecForMap<act_map::GPTraceVoxel>(s, suf, map_dir, bases, vars,
                                               am_opts, esdf_layer, opt.robot_radius,
                                               opt.verbose, opt.only_invalid);
        } else if (s.name == "quad_info") {
          runSpecForMap<act_map::QuadInfoVoxel>(s, suf, map_dir, bases, vars,
                                                am_opts, esdf_layer, opt.robot_radius,
                                                opt.verbose, opt.only_invalid);
        } else if (s.name == "quad_trace") {
          runSpecForMap<act_map::QuadTraceVoxel>(s, suf, map_dir, bases, vars,
                                                 am_opts, esdf_layer, opt.robot_radius,
                                                 opt.verbose, opt.only_invalid);
        }
      }
    }
  } else {
    const std::string label = "custom";
    for (const auto& s : specs) {
      if (!opt.map_type.empty() && s.name != opt.map_type) continue;
      if (s.is_gp) {
        act_map::GPInfoVoxel::setVisApproxFromFolderGP(opt.gp_vis_dir);
        act_map::GPTraceVoxel::setVisApproxFromFolderGP(opt.gp_vis_dir);
      } else {
        act_map::QuadPolyInfoVoxel::setVisApproxQuadVisOpt(am_opts.vis_options_[0]);
        act_map::QuadPolyTraceVoxel::setVisApproxQuadVisOpt(am_opts.vis_options_[0]);
      }
      if (s.name == "gp_info") {
        runSpecForMap<act_map::GPInfoVoxel>(s, label, opt.info_map_dir, bases, vars,
                                            am_opts, esdf_layer, opt.robot_radius,
                                            opt.verbose, opt.only_invalid);
      } else if (s.name == "gp_trace") {
        runSpecForMap<act_map::GPTraceVoxel>(s, label, opt.info_map_dir, bases, vars,
                                             am_opts, esdf_layer, opt.robot_radius,
                                             opt.verbose, opt.only_invalid);
      } else if (s.name == "quad_info") {
        runSpecForMap<act_map::QuadInfoVoxel>(s, label, opt.info_map_dir, bases, vars,
                                              am_opts, esdf_layer, opt.robot_radius,
                                              opt.verbose, opt.only_invalid);
      } else if (s.name == "quad_trace") {
        runSpecForMap<act_map::QuadTraceVoxel>(s, label, opt.info_map_dir, bases, vars,
                                               am_opts, esdf_layer, opt.robot_radius,
                                               opt.verbose, opt.only_invalid);
      }
    }
  }

  return 0;
}
