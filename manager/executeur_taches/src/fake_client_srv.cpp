#include "ros/ros.h"
#include "manager_msg/SetGripper.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_uppdate");
  if (argc != 2)
  {
    ROS_INFO("usage: gripper_uppdate");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<manager_msg::SetGripper>("/fakeRobotino/setGripper");
  manager_msg::SetGripper srv;
  srv.request.state = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("State: No problem");
  }
  else
  {
    ROS_ERROR("Failed to call service gripper_uppdate");
    return 1;
  }

  return 0;
}