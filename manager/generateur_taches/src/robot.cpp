#include "robot.h"
#include <ros/ros.h>

bool robotState(std::vector<comm_msg::Robot> tabRobotInfo, int teamColor, int j, Robot tabRobot[3])
{
	bool noProblem = true;
	for(int k=0; k<tabRobotInfo.size(); k++)
	{
		if(!tabRobotInfo.empty() && tabRobotInfo[k].team_color == teamColor)
		{
			if(tabRobotInfo[k].state == comm_msg::Robot::DISQUALIFIED && tabRobotInfo[k].number == j+1)
			{
				noProblem = false;
				ROS_WARN("Robot %d disqualified",j+1);
			}
			else if(tabRobotInfo[k].state == comm_msg::Robot::MAINTENANCE && tabRobotInfo[k].number == j+1)
			{
				noProblem = false;
				ROS_WARN("Robot %d in maintenance",j+1);
			}
			else if(tabRobot[j].getBusy())
			{
				ROS_INFO("Robot %d is busy",j+1);
			}
			else
			{
				ROS_INFO("Robot %d is ready to receive a new order",j+1);
			}
		}	
	}
	return noProblem;
}
