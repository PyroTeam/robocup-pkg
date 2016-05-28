/**
 * \file 			bumperlistener.h
 * \class			BumperListener
 * \brief			classe représentant l'etat du Bumper
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright       2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef _FINAL_APPROACH__BUMPERLISTENER__H_
#define _FINAL_APPROACH__BUMPERLISTENER__H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class BumperListener 
{
	public:
		BumperListener();
		int getState(){return m_state;}
		void blCallback(const std_msgs::Bool &msg);

	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_blSub;
		bool m_state;

};

#endif // _FINAL_APPROACH__BUMPERLISTENER__H_
