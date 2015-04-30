#include "ros/ros.h"
#include "deplacement_msg/Alarm.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EKF_client");
  if (argc != 1)
  {
    ROS_INFO("usage: 1 to begin EKF, 0 to end");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<deplacement_msg::Alarm>("wake_up");
  deplacement_msg::Alarm srv;

  int tmp = argv[0];
  if (tmp == 1) srv.request.wake_up = deplacement_msg::AlarmRequest::WAKE_UP;
  else          srv.request.wake_up = deplacement_msg::AlarmRequest::SLEEP;

  if (client.call(srv))
  {
    std::cout << "Encore 5 minutes stp..." << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}