/**
 * \file 			bumperlistener.h
 * \class			BumperListener
 * \brief			classe représentant l'etat du Bumper
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef BUMPERLISTENER_H
#define BUMPERLISTENER_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"

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

#endif
