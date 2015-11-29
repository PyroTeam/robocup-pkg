/**
 * \file 			arTagFA.h
 * \class			ArTagFA
 * \brief			classe récupérant les données du topic /ar_pose_markers
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-05-25
 * \copyright		PyroTeam, Polytech-Lille
 */


#ifndef ARTAG_FA_H
#define ARTAG_FA_H

#include <ros/ros.h>
#include <vector>
#include "ar_track_alvar_msgs/AlvarMarkers.h"

class ArTagFA 
{
	public:
		ArTagFA();
		std::vector<int> getId();
		std::vector<float> getPositionX();
		std::vector<float> getPositionZ();
		std::vector<float> getOrientationZ();
		std::vector<float> getDistance();
		bool getFoundId(){return m_foundId;}
		void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_arTagSub;
		std::vector<int> m_id;
		std::vector<float> m_positionX;
		std::vector<float> m_positionZ;
		std::vector<float> m_orientationZ;
		std::vector<float> m_distance;
		bool m_foundId;

};

#endif
