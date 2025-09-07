#pragma once
#include "act_map_exp/quad_rrt.h"

namespace act_map_exp {
template <typename T>
class MyRRT : public QuadRRT<T> {
public:
  MyRRT(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  
  // Override key methods
  void plan() override;
  void setupRRT() override; 
  void extractRRTPathPoses() const override;
  
  // Add new methods if needed
private:
  // Custom members
};
} // namespace act_map_exp
#include "act_map_exp/my_rrt_impl.h"