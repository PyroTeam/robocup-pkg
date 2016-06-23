/**
 * \file        NavigationClientAction.h
 * \class       NavigationClientAction
 * \brief       classe client pour l'action navigation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef NAVIGATIONCLIENTACTION_H
#define NAVIGATIONCLIENTACTION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <deplacement_msg/MoveToPoseAction.h>


class NavigationClientAction
{
	public:
		/* Constructeur */
		NavigationClientAction();

		/* Destructeur */
		virtual  ~NavigationClientAction();

		/* Méthodes */
		int goToAPoint(geometry_msgs::Pose2D dest_point);
};
#endif
