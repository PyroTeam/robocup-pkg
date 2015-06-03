#ifndef ODOM_FA_H
#define ODOM_FA_H

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

class OdomFA 
{
	public:
		OdomFA();
		float getPositionX(){return m_px;}
		float getPositionY(){return m_py;}
		float getOrientationZ(){return m_oz;}
		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_odomSub;
		float m_px;
		float m_py;
		float m_oz;

};

#endif
