#include "final_approach/FinalApproaching.h"
#include "final_approach/Sharps.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <final_approach_msg/FinalApproachingAction.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "finalApproaching_node");
	ros::NodeHandle n;
	ros::Time::init();
	ROS_INFO("Waiting a request");
	FinalApproaching finalApproaching("navigation/finalApproach");
	ros::spin();
}
