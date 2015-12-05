#include "robotInfo.h"
#include <ros/ros.h>
#include "comm_msg/Robot.h"
#include "comm_msg/RobotInfo.h"

void RobotInfo::robotInfoCallback(const comm_msg::RobotInfo::ConstPtr &msg)
{
	m_robots.clear();
	m_robots = msg->robots;
}

RobotInfo::RobotInfo()
{
	m_robotInfoSub = m_nh.subscribe("refBoxComm/RobotInfo",1000,&RobotInfo::robotInfoCallback,this);
}
