/**
 * \file 		action_node.cpp
 * \brief		Programme de test pour recevoir les infos du topic manager_msg::activity
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */

#include "action.h"
#include <ros/ros.h>
#include <iostream>
#include "robot.h"

using namespace std;

void infoRobot(Robot robot)
{
	cout << "\toccupe: "<<(int)robot.getBusy()
		 << " machine: "<<robot.getMachine()
		 << " nb_ordre: "<<robot.getNbOrder()<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_node");
	Robot robot[3];
	Action action;
	ros::Rate loop_rate(10);
    
	while (ros::ok())
    {
		for(int i=0;i<3;i++)
		{
			cout << "number_robot: "<<i<<endl;
			action.updateRobot(robot);
			infoRobot(robot[i]);
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
	return 0;
}
