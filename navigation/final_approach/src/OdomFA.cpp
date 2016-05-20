#include "final_approach/OdomFA.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

void OdomFA::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	m_turn = true;
	m_px = msg->pose.pose.position.x;
	m_py = msg->pose.pose.position.y;
	m_oz = tf::getYaw(msg->pose.pose.orientation);	
}

OdomFA::OdomFA(){
	m_px = 0;
	m_py = 0;
	m_oz = 0;
	m_odomSub = m_nh.subscribe("hardware/odom",1000,&OdomFA::odomCallback,this);        
}

