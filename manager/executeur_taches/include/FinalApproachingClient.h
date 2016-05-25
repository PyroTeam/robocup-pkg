/**
 * \file        FinalApproachingClient.h
 * \class       FinalApproachingClient
 * \brief       classe client pour l'approche finale
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef FINALAPPROACHINGCLIENT_H
#define FINALAPPROACHINGCLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <cstdint>

#include <final_approach_msg/FinalApproachingAction.h>


class FinalApproachingClient
{
	public:
		/* Constructeur */
		FinalApproachingClient();

		/* Déstructeur */
		virtual  ~FinalApproachingClient();

		/* Méthodes */
		void starting(int8_t machineType, int8_t machineSide, int8_t machineParameter);
};
#endif