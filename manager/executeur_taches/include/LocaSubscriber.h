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
#include "deplacement_msg/Machines.h"
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
    void machinesCallback(const deplacement_msg::MachinesConstPtr &msg);
    std::vector<MPS> m_machine;

	private:
		ros::Subscriber m_sub;
};
#endif
