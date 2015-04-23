#include "ros/ros.h"
#include <manager_msg/ReportMachine.h>

bool add(manager_msg::ReportMachine::Request  &req,
         manager_msg::ReportMachine::Response &res)
{
  ROS_INFO("request: name=%s, type=%s, zone = Z%d", req.name.c_str(), req.type.c_str(), req.zone);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "report_machine_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ReportMachine", add);
  ROS_INFO("Ready report a machine");
  ros::spin();

  return 0;
}