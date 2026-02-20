#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <rpg_common/main.h>
#include <rpg_common_ros/params_helper.h>

RPG_COMMON_MAIN
{
  ros::init(argc, argv, "call_generate_mesh");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  const std::string service_name =
      rpg_ros::param<std::string>(pnh, "service_name",
                                  std::string("/voxblox_node/generate_mesh"));
  const double wait_timeout =
      rpg_ros::param<double>(pnh, "wait_timeout", 30.0);
  const double call_delay_sec =
      rpg_ros::param<double>(pnh, "call_delay_sec", 2.0);

  if (call_delay_sec > 0.0)
  {
    ros::Duration(call_delay_sec).sleep();
  }

  bool available = false;
  if (wait_timeout <= 0.0)
  {
    ROS_INFO_STREAM("Waiting for service " << service_name << " (no timeout)...");
    available = ros::service::waitForService(service_name);
  }
  else
  {
    ROS_INFO_STREAM("Waiting for service " << service_name
                                           << " (timeout " << wait_timeout
                                           << "s)...");
    available = ros::service::waitForService(service_name,
                                             ros::Duration(wait_timeout));
  }

  if (!available)
  {
    ROS_ERROR_STREAM("Service not available: " << service_name);
    return 1;
  }

  ros::ServiceClient client =
      nh.serviceClient<std_srvs::Empty>(service_name);
  std_srvs::Empty srv;
  if (!client.call(srv))
  {
    ROS_ERROR_STREAM("Failed to call service: " << service_name);
    return 1;
  }

  ROS_INFO_STREAM("Called service: " << service_name);
  return 0;
}
