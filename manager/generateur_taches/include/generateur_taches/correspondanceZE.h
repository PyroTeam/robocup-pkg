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

	private:

	ros::NodeHandle m_nh;
	ros::Subscriber m_correspondanceZESub;
	std::vector<int> m_usefulZone;


};

#endif
