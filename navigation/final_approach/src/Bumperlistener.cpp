#include "final_approach/Bumperlistener.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

void BumperListener::blCallback(const std_msgs::Bool &msg)
{
	if(msg.data == true)
	{
    	m_state = msg.data;
	}
	//ROS_INFO(" bumper state: %d",m_state);
}

BumperListener::BumperListener(){
	m_state = 0;
	m_blSub = m_nh.subscribe("hardware/bumper", 1, &BumperListener::blCallback, this);
}

