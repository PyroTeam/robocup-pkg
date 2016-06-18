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
#include "deplacement_msg/Landmarks.h" 
#include <vector>
#include <list>

class LocaSubscriber 
{
	public:
		/* Constructeur */	
	 	LocaSubscriber(std::list<int> &lst_unkownZones, std::list<int> &lst_exploredZones, std::list<int> &lst_notExploredZones);

	 	/* Déstructeur */
	    virtual  ~LocaSubscriber();

	    /* Méthodes */
	    void tesCallback(const deplacement_msg::LandmarksConstPtr &msg);
	    int getZone(float x, float y);

	    /* Variable d'instance */
	   	std::list<int> &m_unkownZones, &m_exploredZones, &m_notExploredZones;

	private:
		ros::Subscriber m_sub;
		std::vector<geometry_msgs::Pose2D> m_machinesPose;

};
#endif 