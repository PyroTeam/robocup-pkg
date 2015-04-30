/**
 * \file 		correspondanceZE.h
 * \class		CorrespondanceZE
 * \brief		classe représentant les zones appartenant à l'équipe
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */

#ifndef CORRESPONDANCEZE_H
#define CORRESPONDANCEZE_H

#include "comm_msg/ExplorationInfo.h"
#include <ros/ros.h>
#include <vector>

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
	

	private:

	ros::NodeHandle m_nh;
	ros::Subscriber m_correspondanceZESub;
	std::vector<int> m_usefulZone;
};

#endif
