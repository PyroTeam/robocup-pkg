/**
 * \file        GripperClientSrv.h
 * \class       GripperClientSrv
 * \brief       classe client pour la pince
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef GRIPPERCLIENTSRV_H
#define GRIPPERCLIENTSRV_H


#include "ros/ros.h"
#include "manager_msg/SetGripper.h"
#include <cstdlib>

class GripperClientSrv 
{
	public:
		/* Constructeur */
		GripperClientSrv();

		/* Destructeur */
		virtual  ~GripperClientSrv();

		/* Méthodes */
		bool gripper_uppdate(bool new_state);
};
#endif