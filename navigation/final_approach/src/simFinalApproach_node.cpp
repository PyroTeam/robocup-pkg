#include "final_approach/SimFinalApproach.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <final_approach_msg/FinalApproachingAction.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simFinalApproach");

	SimFinalApproach finalApproach("navigation/finalApproaching_node");

	ros::spin();
}
