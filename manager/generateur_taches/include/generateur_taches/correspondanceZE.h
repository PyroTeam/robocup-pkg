/**
 * \file 		correspondanceZE.h
 * \class		CorrespondanceZE
 * \brief		classe représentant les zones appartenant à l'équipe
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef CORRESPONDANCEZE_H
#define CORRESPONDANCEZE_H

#include "comm_msg/ExplorationInfo.h"
#include "geometry_utils/geometry_utils.h"
#include "common_utils/zone.h"
#include "common_utils/RobotPoseSubscriber.h"
#include "geometry_msgs/Pose2D.h"
#include "LocaSubscriber.h"
#include <ros/ros.h>
#include <vector>
#include <list>
#include <algorithm>

class CorrespondanceZE {

	public:

	CorrespondanceZE();
	~CorrespondanceZE();

	std::vector<int> getUsefulZone();

/**
 *	\brief		Callback permettant de mettre à jour les zones appartenant à l'équipe
 *				grâce au topic comm_msg::ExplorationInfo
 */
	void cZECallback(const comm_msg::ExplorationInfo &msg);
	int getBestZone();
	/* Variable d'instance */
	std::list<int> m_unkownZones, m_exploredZones, m_notExploredZones;
	LocaSubscriber m_locaSub;

	struct s_ZoneDistance {
		int zone;
		double distance;
	};

	private:

	ros::NodeHandle m_nh;
	ros::Subscriber m_correspondanceZESub;
	std::vector<int> m_usefulZone; // A checker si cette variable est tjs utile



};

	bool sortOnDistance (CorrespondanceZE::s_ZoneDistance s1,CorrespondanceZE::s_ZoneDistance s2);
#endif
