#include "docking/Docking.h"
#include "docking/Sharps.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <final_approach_msg/FinalApproachingAction.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "docking_node");
	ros::NodeHandle n;
	ros::Time::init();
	ROS_INFO("Waiting a request");
	Docking Docking("navigation/Docking");
	ros::spin();
}
