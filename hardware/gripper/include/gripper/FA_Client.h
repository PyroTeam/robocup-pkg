/**
 * \file        FA_Client.h
 * \class       FA_Client
 * \brief       classe client pour l'approche finale
 * \author      Danel Thomas
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef FA_CLIENT_H
#define FA_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <final_approach_msg/FinalApproachingAction.h>


class FA_Client
{
	public:
		/* Constructeur */
		FA_Client();

		/* Destructeur */
		virtual  ~FA_Client();

		/* Méthodes */
		int starting(double setpoint);
};
#endif
