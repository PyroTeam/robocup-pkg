/**
 * \file        DockingClient.h
 * \class       DockingClient
 * \brief       classe client pour l'approche finale
 * \author      Danel Thomas
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef DOCKING_CLIENT_H
#define DOCKING_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <final_approach_msg/FinalApproachingAction.h>


class DockingClient
{
	public:
		/* Constructeur */
		DockingClient();

		/* Destructeur */
		virtual  ~DockingClient();

		/* Méthodes */
		int starting(double setpoint);
};
#endif
