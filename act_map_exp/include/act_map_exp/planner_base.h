#pragma once

#include <voxblox_ros/esdf_server.h>
#include <act_map_ros/act_map_server.h>
#include <act_map/information_potential.h>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include "act_map_exp/PlanConfig.h"

namespace act_map_exp
{
using Point = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<Point>;

template <typename T>
class PlannerBase
{
public:
  const static std::string kWorldFrame;
  const static std::string kSaveTwbNm;
  const static std::string kSaveTwcNm;
  const static std::string kSaveTwcUENm;

  PlannerBase() = delete;
  PlannerBase(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  inline virtual void plan()
  {
    // final_Twb_vec_.clear();
    // consec_lower_than_thresh_cnt_ = 0;
    VLOG(1) << "Start RRT plan.";
    const size_t n_iter_prev = rrt_stats_.size();
    for (int cur_iter = 0; cur_iter < max_n_iter_; cur_iter++)
    {
      const int total_iter = cur_iter + 1 + n_iter_prev;
      VLOG(1) << "========= Iter. " << total_iter << " =========";
      ob::PlannerStatus solve_res = planner_->ob::Planner::solve(iter_time_sec_);
      if (solve_res)
      {
        CHECK(pdef_->hasSolution());
        has_valid_solution_ = true;
  
        if (!pdef_->hasExactSolution())
        {
          LOG(WARNING) << "Do not have exact solution.";
        }
        else
        {
          has_exact_solution_ = true;
        }
  
        const int cur_n_iter =
            planner_->as<ompl::geometric::RRTstar>()->numIterations();
        const double cur_best_cost =
            planner_->as<ompl::geometric::RRTstar>()->bestCost().value();
        planner_->as<ompl::geometric::RRTstar>()->getPlannerData(*planner_data_);
  
        VLOG(1) << "=====> Iter. " << total_iter << " succeeds: ";
        VLOG(1) << "- The best cost is " << cur_best_cost;
        VLOG(1) << "- The number of iteration is " << cur_n_iter;
        VLOG(1) << "- Has " << pdef_->getSolutionCount() << " solutions.";
        VLOG(1) << "- Has " << planner_data_->numEdges() << " edges.";
        VLOG(1) << "- Has " << planner_data_->numVertices() << " vertices.";
        const double prev_best_cost = rrt_stats_.lastBestCost();
  
        VLOG(1) << "- Log RRT stats...";
        rrt_stats_.n_iters_.push_back(cur_n_iter);
        rrt_stats_.best_costs_.push_back(cur_best_cost);
        rrt_stats_.n_verts_.push_back(planner_data_->numVertices());
        rrt_stats_.n_edges_.push_back(planner_data_->numEdges());
        act_map::Vec3dVec vertices;
        ompl_utils::getVerticesFromPlannerData(planner_data_, &vertices);
        Eigen::MatrixX3d vert_mat;
        act_map::VecKVecToEigenXK(vertices, &vert_mat);
        rrt_stats_.vertices_.push_back(vert_mat);
        Eigen::MatrixX2i pairs;
        pairs.resize(planner_data_->numEdges(), Eigen::NoChange);
        int pair_i = 0;
        for (size_t start_i = 0; start_i < planner_data_->numVertices();
             start_i++)
        {
          std::vector<unsigned int> end_indices;
          planner_data_->getEdges(start_i, end_indices);
          for (size_t end_i = 0; end_i < end_indices.size(); end_i++)
          {
            pairs(pair_i, 0) = static_cast<int>(start_i);
            pairs(pair_i, 1) = static_cast<int>(end_i);
            pair_i++;
          }
        }
        rrt_stats_.edge_pairs_.push_back(pairs);
  
        if (!std::isfinite(prev_best_cost) || !std::isfinite(cur_best_cost))
        {
          VLOG(1) << "- Either of the prev. or cur. cost is not finite.";
          consec_lower_than_thresh_cnt_ = 0;
        }
        else
        {
          const double ratio =
              std::fabs(cur_best_cost - prev_best_cost) / prev_best_cost;
          VLOG(1) << "- Cost reduction is " << ratio;
          if (ratio < converge_ratio_)
          {
            consec_lower_than_thresh_cnt_++;
          }
          else
          {
            consec_lower_than_thresh_cnt_ = 0;
          }
        }
  
        constexpr int kMinConsecRed = 2;
        VLOG(1) << "- consecutive reduction cnt. "
                << consec_lower_than_thresh_cnt_;
        if (cur_iter >= min_n_iter_)
        {
          if (consec_lower_than_thresh_cnt_ >= kMinConsecRed)
          {
            VLOG(1) << "Consecutive " << kMinConsecRed
                    << " iterations reduce the cost under threshold, stop.";
            break;
          }
        }
  
        if (viz_every_outer_iter_)
        {
          VLOG(1) << "Visualize current results...";
          extractRRTPathPoses();
          visualize();
        }
      }
      else
      {
        LOG(WARNING) << "XXXXX> Solve failed for iter " << total_iter;
      }
    }
  
    if (has_valid_solution_)
    {
      VLOG(1) << "Solve successfully!";
      extractRRTPathPoses();
      VLOG(1) << "Get " << final_Twb_vec_.size() << " vertices.";
    }
    LOG(WARNING) << "Base planner plan() function called.";
  }

  inline virtual void visualize()
  {
    LOG(WARNING) << "Base planner visualize() function called.";
  }

  inline virtual void saveResults()
  {
    LOG(WARNING) << "Base planner saveResults() function called.";
  }

  inline virtual void resetPlannerState()
  {
    LOG(WARNING) << "Base planner resetPlannerState() function called";
  }

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  inline virtual bool setPlannerStateCallback(PlanConfig::Request& req,
                                              PlanConfig::Response& res)
  {
    LOG(WARNING) << "Base planner setPlanner() function called";
    return false;
  }

  bool planVisSaveCallback(std_srvs::Empty::Request& req,
                           std_srvs::Empty::Response& res);

  bool resetPlannerCallback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& res);

  bool publishVisualizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);

  // common services
  ros::ServiceServer set_planner_state_srv_;
  ros::ServiceServer plan_vis_save_srv_;
  ros::ServiceServer reset_planner_srv_;
  ros::ServiceServer pub_vis_srv_;

  // basic trajectory and markers
  ros::Publisher traj_orient_pub_;
  ros::Publisher traj_pos_pub_;
  ros::Publisher general_marker_pub_;

  // information potential
  act_map::InfoPotentialPtr<T> info_pot_ptr_;

  //
  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  std::shared_ptr<act_map_ros::ActMapServer<T>> act_map_server_;

  // services for convenience
  ros::ServiceServer update_vis_srv_;
  ros::ServiceServer clear_map_srv;

  void createInfoPotential();

  bool updateAllVisualizationCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& res);
  bool clearAllMapCallback(std_srvs::Empty::Request& req,
                           std_srvs::Empty::Response& res);

  // utilities
  bool hasValidESDFMap() const;
  bool hasValidInfoMap() const;
};

template <typename T>
const std::string PlannerBase<T>::kWorldFrame = std::string("world");

template <typename T>
const std::string PlannerBase<T>::kSaveTwbNm = std::string("stamped_Twb.txt");

template <typename T>
const std::string PlannerBase<T>::kSaveTwcNm = std::string("stamped_Twc.txt");

template <typename T>
const std::string
    PlannerBase<T>::kSaveTwcUENm = std::string("stamped_Twc_ue.txt");

template <typename T>
PlannerBase<T>::PlannerBase(const ros::NodeHandle& nh,
                            const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  set_planner_state_srv_ = this->pnh_.advertiseService(
      "set_planner_state", &PlannerBase::setPlannerStateCallback, this);
  plan_vis_save_srv_ = this->pnh_.advertiseService(
      "plan_vis_save", &PlannerBase::planVisSaveCallback, this);
  reset_planner_srv_ = this->pnh_.advertiseService(
      "reset_planner", &PlannerBase::resetPlannerCallback, this);
  pub_vis_srv_ = this->pnh_.advertiseService(
      "publish_traj_vis", &PlannerBase::publishVisualizationCallback, this);

  esdf_server_.reset(new voxblox::EsdfServer(nh_, pnh_));
  act_map_server_.reset(new act_map_ros::ActMapServer<T>(nh_, pnh_));

  update_vis_srv_ = pnh_.advertiseService(
      "update_all_vis", &PlannerBase::updateAllVisualizationCallback, this);
  clear_map_srv = pnh_.advertiseService(
      "clear_all_maps", &PlannerBase::clearAllMapCallback, this);

  traj_orient_pub_ =
      pnh_.advertise<visualization_msgs::MarkerArray>("traj_orient", 50);
  traj_pos_pub_ = pnh_.advertise<PointCloud>("traj_pos", 50);
  general_marker_pub_ =
      pnh_.advertise<visualization_msgs::Marker>("general_markers", 10);

  const bool info_metric_use_log_det =
      rpg_ros::param<bool>(this->pnh_, "info_metric_use_log_det", true);
  act_map::kInfoMetricUseLogDet = info_metric_use_log_det;
  createInfoPotential();
}

template <typename T>
void PlannerBase<T>::createInfoPotential()
{
  act_map::InfoPotentialOptions ip_options =
      act_map_ros::readInfoPotentialOptions(this->pnh_);
  VLOG(1) << "Creating potential function for information...";
  info_pot_ptr_.reset(new act_map::InformationPotential<T>(
      ip_options,
      this->act_map_server_->getActMapCRef().options_.vis_options_.at(0)));
  VLOG(1) << (*info_pot_ptr_);
}

template <typename T>
bool PlannerBase<T>::updateAllVisualizationCallback(
    std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  esdf_server_->publishPointclouds();
  act_map_server_->updateVisualization();
  return true;
}

template <typename T>
bool PlannerBase<T>::clearAllMapCallback(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res)
{
  esdf_server_->clear();
  act_map_server_->getActMapRef().clearMap();
  return true;
}

template <typename T>
bool PlannerBase<T>::hasValidESDFMap() const
{
  return esdf_server_->getEsdfMapPtr()
             ->getEsdfLayerPtr()
             ->getNumberOfAllocatedBlocks() != 0;
}

template <typename T>
bool PlannerBase<T>::hasValidInfoMap() const
{
  return act_map_server_->getActMapCRef()
             .kerLayerCRef()
             .getNumberOfAllocatedBlocks() != 0;
}

template <typename T>
bool PlannerBase<T>::resetPlannerCallback(std_srvs::Empty::Request& /*req*/,
                                          std_srvs::Empty::Response& /*res*/)
{
  resetPlannerState();
  return true;
}

template <typename T>
bool PlannerBase<T>::publishVisualizationCallback(
    std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
{
  visualize();
  return true;
}

template <typename T>
bool PlannerBase<T>::planVisSaveCallback(std_srvs::Empty::Request& /*req*/,
                                         std_srvs::Empty::Response& /*res*/)
{
  plan();
  visualize();
  saveResults();
  return true;
}

}  // namespace act_map_exp
