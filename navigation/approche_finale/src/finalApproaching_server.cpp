#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <manager_msg/finalApproachingAction.h>
#include "finalApproaching.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "finalApproaching_node");
	ros::NodeHandle n;
	ros::Time::init();
	ROS_INFO("En attente d une demande d action");
  finalApproaching finalApproaching(ros::this_node::getName());
  ros::spin();
}
