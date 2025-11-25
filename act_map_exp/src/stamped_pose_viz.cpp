#include "act_map_exp/stamped_pose_viz.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <unrealcv_bridge/ue_utils.hpp>

namespace act_map_exp
{

namespace
{
inline std::string getTrimmedLine(const std::string& line)
{
  const auto first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
  {
    return std::string();
  }
  return line.substr(first);
}

bool isCommentOrEmpty(const std::string& trimmed)
{
  return trimmed.empty() ||
         trimmed.front() == '#';
}
}  // namespace

bool loadStampedUePoses(const std::string& file_path,
                        rpg::PoseVec* poses,
                        std::vector<Eigen::Vector3d>* view_dirs,
                        std::string* error_msg)
{
  if (!poses || !view_dirs)
  {
    if (error_msg)
    {
      *error_msg = "output containers must not be null";
    }
    return false;
  }

  if (file_path.empty())
  {
    if (error_msg)
    {
      *error_msg = "file path empty";
    }
    return false;
  }

  std::ifstream stream(file_path);
  if (!stream.is_open())
  {
    if (error_msg)
    {
      *error_msg = "failed to open " + file_path;
    }
    return false;
  }

  poses->clear();
  view_dirs->clear();

  std::string line;
  while (std::getline(stream, line))
  {
    const std::string trimmed = getTrimmedLine(line);
    if (isCommentOrEmpty(trimmed))
    {
      continue;
    }

    std::istringstream iss(trimmed);
    double idx = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;
    if (!(iss >> idx >> x >> y >> z >> pitch >> yaw >> roll))
    {
      continue;
    }

    unrealcv_bridge::UEPose uep;
    uep.x = x;
    uep.y = y;
    uep.z = z;
    uep.pitch = pitch;
    uep.yaw = yaw;
    uep.roll = roll;

    rpg::Pose Twc;
    uep.toTwc(&Twc);

    poses->emplace_back(Twc);

    Eigen::Vector3d view_dir = -Twc.getRotation().getRotationMatrix().col(2);
    const double norm = view_dir.norm();
    if (norm > 1e-9)
    {
      view_dir /= norm;
    }
    view_dirs->emplace_back(view_dir);
  }

  if (poses->empty())
  {
    if (error_msg)
    {
      *error_msg = "no poses parsed from " + file_path;
    }
    return false;
  }

  return true;
}

double computeStampedArrowLength(const rpg::PoseVec& poses)
{
  if (poses.empty())
  {
    return 0.5;
  }
  Eigen::Vector3d min_pt = poses.front().getPosition();
  Eigen::Vector3d max_pt = min_pt;
  for (const rpg::Pose& pose : poses)
  {
    const Eigen::Vector3d pos = pose.getPosition();
    min_pt = min_pt.cwiseMin(pos);
    max_pt = max_pt.cwiseMax(pos);
  }
  const double diag = (max_pt - min_pt).norm();
  return std::max(0.5, diag * 0.2);
}

visualization_msgs::Marker createStampedPathMarker(
    const rpg::PoseVec& poses, const std::string& frame,
    const std_msgs::ColorRGBA& color, const std::string& ns, const int id,
    const double line_width)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = static_cast<float>(line_width);
  marker.pose.orientation.w = 1.0;
  marker.color = color;
  marker.color.a = color.a <= 0.0f ? 1.0f : color.a;

  for (const rpg::Pose& pose : poses)
  {
    geometry_msgs::Point point;
    point.x = pose.getPosition().x();
    point.y = pose.getPosition().y();
    point.z = pose.getPosition().z();
    marker.points.push_back(point);
  }
  return marker;
}

visualization_msgs::Marker createStampedViewMarker(
    const rpg::PoseVec& poses,
    const std::vector<Eigen::Vector3d>& view_dirs,
    const std::string& frame, const std_msgs::ColorRGBA& color,
    const std::string& ns, const int id, const double arrow_length,
    const double line_width)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = static_cast<float>(line_width);
  marker.pose.orientation.w = 1.0;
  marker.color = color;
  marker.color.a = color.a <= 0.0f ? 1.0f : color.a;

  if (poses.size() != view_dirs.size())
  {
    return marker;
  }

  for (size_t idx = 0; idx < poses.size(); idx++)
  {
    const Eigen::Vector3d end_point =
        poses[idx].getPosition() + view_dirs[idx] * arrow_length;
    geometry_msgs::Point start;
    start.x = poses[idx].getPosition().x();
    start.y = poses[idx].getPosition().y();
    start.z = poses[idx].getPosition().z();

    geometry_msgs::Point end;
    end.x = end_point.x();
    end.y = end_point.y();
    end.z = end_point.z();

    marker.points.push_back(start);
    marker.points.push_back(end);
  }

  return marker;
}

}  // namespace act_map_exp
