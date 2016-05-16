#include "FinalApproaching.h"
#include "Sharps.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <manager_msg/FinalApproachingAction.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "finalApproaching_node");
	ros::NodeHandle n;
	ros::Time::init();
	ROS_INFO("Waiting a request");
	//Sharps sharps;
	FinalApproaching finalApproaching("navigation/" + ros::this_node::getName());
	ros::spin();
}
