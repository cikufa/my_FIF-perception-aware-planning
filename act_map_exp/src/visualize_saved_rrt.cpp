#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rpg_common/pose.h>

namespace
{
using Point = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<Point>;

std_msgs::ColorRGBA color(const double r, const double g, const double b,
                          const double a = 1.0)
{
  std_msgs::ColorRGBA c;
  c.r = static_cast<float>(r);
  c.g = static_cast<float>(g);
  c.b = static_cast<float>(b);
  c.a = static_cast<float>(a);
  return c;
}

double lerp(const double a, const double b, const double t)
{
  return a + (b - a) * t;
}

void sampleIndices(const std::size_t max_samples, const std::size_t total,
                   std::vector<std::size_t>* out)
{
  out->clear();
  if (total == 0 || max_samples == 0)
  {
    return;
  }
  if (total <= max_samples)
  {
    out->reserve(total);
    for (std::size_t i = 0; i < total; ++i)
    {
      out->push_back(i);
    }
    return;
  }

  const double step = static_cast<double>(total - 1) /
                      static_cast<double>(max_samples - 1);
  out->reserve(max_samples);
  for (std::size_t i = 0; i < max_samples; ++i)
  {
    const std::size_t idx = static_cast<std::size_t>(std::round(i * step));
    out->push_back(std::min(idx, total - 1));
  }
}

double computePathLength(const rpg::PoseVec& poses)
{
  double length = 0.0;
  for (std::size_t i = 1; i < poses.size(); ++i)
  {
    length += (poses[i].getPosition() - poses[i - 1].getPosition()).norm();
  }
  return length;
}

bool loadStampedTwc(const std::string& file_path, rpg::PoseVec* poses)
{
  if (!poses)
  {
    ROS_ERROR("Output pose container is null.");
    return false;
  }

  poses->clear();
  std::ifstream stream(file_path);
  if (!stream.is_open())
  {
    ROS_ERROR_STREAM("Failed to open pose file: " << file_path);
    return false;
  }

  std::string line;
  while (std::getline(stream, line))
  {
    const std::size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos)
    {
      line = line.substr(0, comment_pos);
    }

    std::istringstream iss(line);
    double time = 0.0;
    if (!(iss >> time))
    {
      continue;
    }

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    bool complete = true;
    for (int i = 0; i < 16; ++i)
    {
      double v = 0.0;
      if (!(iss >> v))
      {
        complete = false;
        break;
      }
      T(i / 4, i % 4) = v;
    }
    if (!complete)
    {
      ROS_WARN_STREAM("Incomplete line in stamped_Twc: " << line);
      continue;
    }
    poses->emplace_back(rpg::Pose(T));
  }

  if (poses->empty())
  {
    ROS_ERROR_STREAM("No poses read from " << file_path);
    return false;
  }

  return true;
}

void publishTrajectoryPoints(const rpg::PoseVec& Tw_vec,
                             const ros::Publisher& traj_pos_pub,
                             const Eigen::Vector3d& rgb,
                             const std::string& frame)
{
  if (traj_pos_pub.getNumSubscribers() == 0)
  {
    return;
  }
  PointCloud traj_pos_pc;
  pcl_conversions::toPCL(ros::Time::now(), traj_pos_pc.header.stamp);
  traj_pos_pc.header.frame_id = frame;
  traj_pos_pc.reserve(Tw_vec.size());
  for (const rpg::Pose& Tw : Tw_vec)
  {
    Point pt;
    pt.x = static_cast<float>(Tw.getPosition().x());
    pt.y = static_cast<float>(Tw.getPosition().y());
    pt.z = static_cast<float>(Tw.getPosition().z());
    pt.r = static_cast<uint8_t>(rgb(0) * 255);
    pt.g = static_cast<uint8_t>(rgb(1) * 255);
    pt.b = static_cast<uint8_t>(rgb(2) * 255);
    traj_pos_pc.push_back(pt);
  }
  traj_pos_pub.publish(traj_pos_pc);
}

void publishTrajectoryOrientation(const rpg::PoseVec& Twc_vec,
                                  const ros::Publisher& orient_pub,
                                  const Eigen::Vector3d& rgb,
                                  const std::string& frame,
                                  const std::string& ns_prefix,
                                  const int base_id)
{
  if (orient_pub.getNumSubscribers() == 0 || Twc_vec.empty())
  {
    return;
  }

  constexpr std::size_t kMaxMarkers = 10u;
  std::vector<std::size_t> idx;
  sampleIndices(kMaxMarkers, Twc_vec.size(), &idx);

  const double base_len =
      std::max(0.1, computePathLength(Twc_vec) /
                        static_cast<double>(std::max<std::size_t>(idx.size(), 1)));

  visualization_msgs::MarkerArray markers;
  int id = base_id;
  for (const std::size_t i : idx)
  {
    const rpg::Pose& Twc = Twc_vec.at(i);

    visualization_msgs::Marker cam;
    cam.header.frame_id = frame;
    cam.header.stamp = ros::Time::now();
    cam.ns = ns_prefix;
    cam.id = id++;
    cam.type = visualization_msgs::Marker::ARROW;
    cam.action = visualization_msgs::Marker::ADD;
    cam.pose.position.x = Twc.getPosition().x();
    cam.pose.position.y = Twc.getPosition().y();
    cam.pose.position.z = Twc.getPosition().z();
    cam.pose.orientation.w = Twc.getRotation().w();
    cam.pose.orientation.x = Twc.getRotation().x();
    cam.pose.orientation.y = Twc.getRotation().y();
    cam.pose.orientation.z = Twc.getRotation().z();
    cam.scale.x = base_len;
    cam.scale.y = base_len * 0.15;
    cam.scale.z = base_len * 0.15;
    cam.color = color(rgb(0), rgb(1), rgb(2), 0.9);
    markers.markers.push_back(cam);
  }

  orient_pub.publish(markers);
}

void visualizeTrajectory(const rpg::PoseVec& Twb_vec,
                         const ros::Publisher& traj_pos_pub,
                         const ros::Publisher& traj_orient_pub,
                         const Eigen::Vector3d& rgb,
                         const std::string& frame,
                         const std::string& ns_prefix,
                         const int base_id)
{
  publishTrajectoryPoints(Twb_vec, traj_pos_pub, rgb, frame);
  publishTrajectoryOrientation(Twb_vec, traj_orient_pub, rgb, frame, ns_prefix,
                               base_id);
}

void visualizePath(const rpg::PoseVec& poses,
                   const std_msgs::ColorRGBA& start_c,
                   const std_msgs::ColorRGBA& end_c,
                   const ros::Publisher& marker_pub,
                   const int id,
                   const std::string& frame,
                   const std::string& ns)
{
  if (poses.empty() || marker_pub.getNumSubscribers() == 0)
  {
    return;
  }

  visualization_msgs::Marker line;
  line.ns = ns;
  line.id = id;
  line.header.frame_id = frame;
  line.header.stamp = ros::Time::now();
  line.pose.orientation.w = 1.0;
  line.action = visualization_msgs::Marker::ADD;
  line.type = visualization_msgs::Marker::LINE_STRIP;

  const double width = std::max(0.01, computePathLength(poses) * 0.01);
  line.scale.x = width;

  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = poses[i].getPosition().x();
    pt.y = poses[i].getPosition().y();
    pt.z = poses[i].getPosition().z();
    line.points.push_back(pt);

    const double t = poses.size() > 1 ? static_cast<double>(i) /
                                            static_cast<double>(poses.size() - 1)
                                      : 0.0;
    std_msgs::ColorRGBA c;
    c.r = static_cast<float>(lerp(start_c.r, end_c.r, t));
    c.g = static_cast<float>(lerp(start_c.g, end_c.g, t));
    c.b = static_cast<float>(lerp(start_c.b, end_c.b, t));
    c.a = static_cast<float>(lerp(start_c.a, end_c.a, t));
    line.colors.push_back(c);
  }

  marker_pub.publish(line);
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_saved_rrt");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string pose_file_a;
  std::string pose_file_b;
  pnh.param<std::string>("pose_file_a", pose_file_a, std::string());
  pnh.param<std::string>("pose_file_b", pose_file_b, std::string());
  double r_a = 0.0, g_a = 1.0, b_a = 0.0;
  double r_b = 1.0, g_b = 0.5, b_b = 0.0;
  pnh.param("r_a", r_a, r_a);
  pnh.param("g_a", g_a, g_a);
  pnh.param("b_a", b_a, b_a);
  pnh.param("r_b", r_b, r_b);
  pnh.param("g_b", g_b, g_b);
  pnh.param("b_b", b_b, b_b);

  if (pose_file_a.empty() && pose_file_b.empty())
  {
    ROS_ERROR("At least one of ~pose_file_a or ~pose_file_b must be set.");
    return 1;
  }

  std::string frame_id;
  pnh.param<std::string>("frame", frame_id, std::string("world"));

  ros::Publisher traj_pos_pub =
      pnh.advertise<PointCloud>("traj_pos", 1, true);
  ros::Publisher traj_orient_pub =
      pnh.advertise<visualization_msgs::MarkerArray>("traj_orient", 1, true);
  ros::Publisher marker_pub =
      pnh.advertise<visualization_msgs::Marker>("general_markers", 1, true);

  // Allow time for subscribers to connect to latched publishers.
  ros::Duration(0.3).sleep();

  if (!pose_file_a.empty())
  {
    rpg::PoseVec Twc_vec_a;
    if (loadStampedTwc(pose_file_a, &Twc_vec_a))
    {
      const Eigen::Vector3d rgb_a(r_a, g_a, b_a);
      visualizeTrajectory(Twc_vec_a, traj_pos_pub, traj_orient_pub, rgb_a,
                          frame_id, "traj_a", 0);
      visualizePath(Twc_vec_a, color(rgb_a(0), rgb_a(1), rgb_a(2)),
                    color(rgb_a(0), rgb_a(1), rgb_a(2)),
                    marker_pub, 0, frame_id, "saved_rrt_path_a");
      ROS_INFO_STREAM("Published " << Twc_vec_a.size() << " poses from "
                                   << pose_file_a << " to frame " << frame_id
                                   << ".");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to load " << pose_file_a);
    }
  }

  if (!pose_file_b.empty())
  {
    rpg::PoseVec Twc_vec_b;
    if (loadStampedTwc(pose_file_b, &Twc_vec_b))
    {
      const Eigen::Vector3d rgb_b(r_b, g_b, b_b);
      visualizeTrajectory(Twc_vec_b, traj_pos_pub, traj_orient_pub, rgb_b,
                          frame_id, "traj_b", 1000);
      visualizePath(Twc_vec_b, color(rgb_b(0), rgb_b(1), rgb_b(2)),
                    color(rgb_b(0), rgb_b(1), rgb_b(2)),
                    marker_pub, 1, frame_id, "saved_rrt_path_b");
      ROS_INFO_STREAM("Published " << Twc_vec_b.size() << " poses from "
                                   << pose_file_b << " to frame " << frame_id
                                   << ".");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to load " << pose_file_b);
    }
  }

  ros::spin();
  return 0;
}
