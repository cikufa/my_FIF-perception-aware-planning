#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <rpg_common/pose.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace act_map_exp
{

bool loadStampedUePoses(const std::string& file_path,
                        rpg::PoseVec* poses,
                        std::vector<Eigen::Vector3d>* view_dirs,
                        std::string* error_msg);

double computeStampedArrowLength(const rpg::PoseVec& poses);

visualization_msgs::Marker createStampedPathMarker(
    const rpg::PoseVec& poses, const std::string& frame,
    const std_msgs::ColorRGBA& color, const std::string& ns, const int id,
    const double line_width = 0.04);

visualization_msgs::Marker createStampedViewMarker(
    const rpg::PoseVec& poses,
    const std::vector<Eigen::Vector3d>& view_dirs,
    const std::string& frame, const std_msgs::ColorRGBA& color,
    const std::string& ns, const int id,
    const double arrow_length, const double line_width = 0.02);

}  // namespace act_map_exp
