/**
 * \file        LocaSubscriber.h
 * \class       LocaSubscriber
 * \brief       classe pour s'abonner au Topic de la localisation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef LOCASUBSCRIBER_H
#define LOCASUBSCRIBER_H

#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h" 
#include "MPS.h"
#include <vector>

class LocaSubscriber 
{
	public:
		/* Constructeur */	
	 	LocaSubscriber();

	 	/* Déstructeur */
	    virtual  ~LocaSubscriber();

	    /* Méthodes */
	    void tesCallback(const deplacement_msg::LandmarksConstPtr &msg);
	    int getArea(float x, float y);
	    std::vector<geometry_msgs::Pose2D> tab_machine;
	    std::vector<MPS> m_machine;

	private:
		ros::Subscriber m_sub;
};
#endif 

