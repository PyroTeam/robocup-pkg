/**
 * \file 		activityTalker_node.cpp
 * \brief		Programme de test pour simuler l envoi du topic manager_msg::activity
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */
 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "manager_msg/activity.h" 

#include <sstream>

using namespace manager_msg;

int main(int argc, char **argv)
{
	ROS_INFO("Starting node emetteur_topic");
	ros::init(argc, argv, "emetteur_topic");
	ros::NodeHandle n;
	ros::Publisher activite = n.advertise<activity>("manager/task_exec_state", 1000);
	ros::Rate loop_rate(50);
	int count = 0;
	while (ros::ok())
	{
		activity msg;
		msg.nb_robot = 1;
		msg.state = activity::IN_PROGRESS; /* Par exemple */
		if(count>20)
		{
			msg.nb_robot = 2;
		}
		activite.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
	++count;
	}
	return 0;
}
