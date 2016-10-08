/**
 * \file 			robotInfo.h
 * \class			robotInfo
 * \brief			classe récupérant l etat des robots du topic 
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-06-23
 * \copyright       2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef ROBOTINFO_H
#define ROBOTINFO_H

#include <ros/ros.h>
#include <vector>
#include "comm_msg/Robot.h"
#include "comm_msg/RobotInfo.h"


class RobotInfo{

public:

	RobotInfo();
	
	std::vector<comm_msg::Robot> getRobots(){return m_robots;}
	
	void robotInfoCallback(const comm_msg::RobotInfo::ConstPtr &msg);
	
	  
private:

	ros::NodeHandle m_nh;
    ros::Subscriber m_robotInfoSub;

    std::vector<comm_msg::Robot> m_robots;

};

#endif
