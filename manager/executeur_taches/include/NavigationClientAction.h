/**
 * \file        NavigationClientAction.h
 * \class       NavigationClientAction
 * \brief       classe client pour l'action navigation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef NAVIGATIONCLIENTACTION_H
#define NAVIGATIONCLIENTACTION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <deplacement_msg/MoveToPoseAction.h>
#include <deplacement_msg/finalApproaching.h>


class NavigationClientAction
{
	public:
		/* Constructeur */
		NavigationClientAction();

		/* Déstructeur */
		virtual  ~NavigationClientAction();

		/* Méthodes */
		deplacement_msg::finalApproaching setFinalApproachingData(int8_t machineType, int8_t machineSide, int8_t machineParameter);
		bool getSuccess();
		int16_t getError();
		int16_t getStatus();
		void navigate(geometry_msgs::Pose2D dest_point, bool fa, bool fast, bool puckInGripper, bool goAway, int8_t machineType, int8_t machineSide, int8_t machineParameter);

	private:
		bool m_success;
		int16_t m_error;
		int16_t m_status;
};
#endif
