/**
 * \file        FeuClientAction.h
 * \class       FeuClientAction
 * \brief       classe client pour le noeud Feu
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef FEUCLIENTACTION_H
#define FEUCLIENTACTION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <trait_im_msg/processLightSignalAction.h>
#include <comm_msg/LightSpec.h>


class FeuClientAction
{
	public:
		/* Constructeur */
		FeuClientAction();

		/* Déstructeur */
		virtual  ~FeuClientAction();

		/* Méthodes */
		void lightsStates(std::vector<comm_msg::LightSpec> &m_lightSpec);
		//private:
		//manager_msg::LightSpec m_lightSpec[];
};
#endif
