/**
 * \file 			odomFA.h
 * \class			OdomFA
 * \brief			classe récupérant les données du topic /odom
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-05-25
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef _FINAL_APPROACH__ODOM_FA__H_
#define _FINAL_APPROACH__ODOM_FA__H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class OdomFA
{
	public:
		OdomFA();
		bool getTurn(){return m_turn;}
		float getPositionX(){return m_px;}
		float getPositionY(){return m_py;}
		float getOrientationZ(){return m_oz;}
		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_odomSub;
		bool m_turn=false;
		float m_px;
		float m_py;
		float m_oz;

};

#endif // _FINAL_APPROACH__ODOM_FA__H_
