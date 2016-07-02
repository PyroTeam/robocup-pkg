/**
 * \file        LocaSubscriber.h
 * \class       LocaSubscriber
 * \brief       classe pour s'abonner au Topic de la localisation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
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

	 	/* Destructeur */
    virtual  ~LocaSubscriber();

    /* Méthodes */
    void machinesCallback(const deplacement_msg::MachinesConstPtr &msg);
    void spin();

    std::vector<MPS> machines(){return m_machines;};
    bool haveAllTheMachines(){return m_explo_completed;};

	private:
		ros::Subscriber m_sub;
    std::vector<MPS> m_machines;
    bool m_explo_completed;
};
#endif
