#include "act_map_exp/my_rrt.h"

namespace act_map_exp {
template <typename T>
MyRRT<T>::MyRRT(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : QuadRRT<T>(nh, pnh) {

  // Custom initialization
}

template <typename T>
void MyRRT<T>::plan() {
  // Custom planning logic
  
  QuadRRT<T>::plan();  // Call parent if needed
}

template <typename T>
void MyRRT<T>::setupRRT() {
  VLOG(1) << "\nTTTT  TTTT   TTTTT    TTTTTTT      TTTTTT       TTTTTTTT\n shekoufeh rrt (3dof without fim)\nTTTT  TTTT   TTTTT    TTTTTTT      TTTTTT       TTTTTTTT\nTTTT  TTTT   TTTTT    TTTTTTT      TTTTTT       TTTTTTTT\n";

  ob::StateSpacePtr pos_ss(new ob::RealVectorStateSpace(3));
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, this->min_x_);
  bounds.setHigh(0, this->max_x_);
  bounds.setLow(1, this->min_y_);
  bounds.setHigh(1, this->max_y_);
  bounds.setLow(2, this->min_z_);
  bounds.setHigh(2, this->max_z_);
  pos_ss->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  this->flat_ss_ = pos_ss;
  this->flat_si_.reset(new ob::SpaceInformation(this->flat_ss_));

  // ESDF-only Validity Checking
  VLOG(1) << " using ESDF for validity check.";
  if (this->hasValidESDFMap()) {
    this->flat_si_->setStateValidityChecker(
        ob::StateValidityCheckerPtr(new ompl_utils::ESDFChecker(
            this->flat_si_,
            std::const_pointer_cast<const voxblox::EsdfServer>(
                this->esdf_server_)
                ->getEsdfMapPtr(),
            this->robot_radius_)));
  } else {
    LOG(WARNING) << "No ESDF map found, using dummy checker";
    this->flat_si_->setStateValidityChecker(
        ob::StateValidityCheckerPtr(new ompl_utils::DummyChecker(this->flat_si_)));
  }

  this->flat_si_->setStateValidityCheckingResolution(this->calValidityResolution());
  this->flat_si_->setup();

  // Problem Definition
  VLOG(1) << "- problem definition";
  VLOG(1) << "  distance between goal and start positions: "
          << (this->start_pos_ - this->end_pos_).norm(); //ADDED:
  this->pdef_.reset(new ob::ProblemDefinition(this->flat_si_));
  
  ob::ScopedState<> start(this->flat_ss_);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = this->start_pos_.x();
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = this->start_pos_.y();
  start->as<ob::RealVectorStateSpace::StateType>()->values[2] = this->start_pos_.z();
  ob::ScopedState<> goal(this->flat_ss_);
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = this->end_pos_.x();
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = this->end_pos_.y();
  goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = this->end_pos_.z();
  this->pdef_->setStartAndGoalStates(start, goal, 1.0);

  VLOG(1) << "- planner";
  // Cost Function (Distance only)
  ob::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(this->flat_si_));
  this->pdef_->setOptimizationObjective(obj);
  obj->print(std::cout);

  // 6. Planner Setup (RRT* by default)
  this->planner_.reset(new ompl::geometric::RRTstar(this->flat_si_));
  if (this->rrt_range_ > 0) {
    std::dynamic_pointer_cast<ompl::geometric::RRTstar>(this->planner_)->setRange(this->rrt_range_);
  }
  this->planner_->setProblemDefinition(this->pdef_);
  this->planner_->setup();

  this->planner_data_.reset(new ob::PlannerData(this->flat_si_));

  VLOG(1) << "=== Summary of problem and planner ===";
  VLOG(1) << "- problem def.:";
  this->pdef_->print();
  VLOG(1) << "- planner settings: ";
  this->planner_->printSettings(std::cout);
  VLOG(1) << "- planner prop.:";
  this->planner_->printProperties(std::cout);


  // Custom setup (e.g., different state space or cost functions)
  // QuadRRT<T>::setupRRT();  // Call parent if needed
}

template <typename T>
void MyRRT<T>::extractRRTPathPoses() const {
  VLOG(1) << "\nTTTT  TTTT   TTTTT    TTTTTTT      TTTTTT       TTTTTTTT\n shekoufeh rrt (extractRRTPathPoses())\nTTTT  TTTT   TTTTT    TTTTTTT      TTTTTT       TTTTTTTT\nTTTT  TTTT   TTTTT    TTTTTTT      TTTTTT       TTTTTTTT\n";

  if (!this->has_valid_solution_) {
    return;
  }

  this->final_Twb_vec_.clear();
  ob::PathPtr path = this->pdef_->getSolutionPath();
  const std::vector<ob::State*>& states =
      path->as<ompl::geometric::PathGeometric>()->getStates();
  this->final_Twb_vec_.reserve(states.size());
  
  for (const ob::State* s : states) {
    const auto* pos = s->as<ob::RealVectorStateSpace::StateType>();
    // this->final_Twb_vec_.emplace_back(
    //     rpg::Pose(rpg::Rotation::Identity(), 
    //              Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2])));
    this->final_Twb_vec_.emplace_back(
      rpg::Pose(rpg::Rotation(), 
               Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2])));
  }
}

} // namespace act_map_exp