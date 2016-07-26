/**
 * \file        LocaSubscriber.h
 * \class       LocaSubscriber
 * \brief       classe pour s'abonner au Topic de la localisation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2016-06-18
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef LOCASUBSCRIBER_H
#define LOCASUBSCRIBER_H

#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Machines.h"
#include "common_utils/Zone.h"
#include <vector>
#include <list>
#include <algorithm>

class LocaSubscriber
{
	public:
		/* Constructeur */
	 	LocaSubscriber(std::list<int> &lst_unkownZones, std::list<int> &lst_exploredZones, std::list<int> &lst_notExploredZones);

	 	/* Déstructeur */
	    virtual  ~LocaSubscriber();

	    /* Méthodes */
	    void tesCallback(const deplacement_msg::MachinesConstPtr &msg);
			bool foundInUnkown(int z);
			bool foundInExplored(int z);
			bool foundInNotExplored(int z);
			void pushToExploredList(int z);
			void removeFromUnkown(int z);
			void removeFromExplore(int z);
			void removeFromNotExplored(int z);
			/* Variable d'instance */
	   	std::list<int> &m_unkownZones, &m_exploredZones, &m_notExploredZones;

	private:
		ros::Subscriber m_sub;
		std::vector<deplacement_msg::Machine> m_machinesPose;

};
#endif
