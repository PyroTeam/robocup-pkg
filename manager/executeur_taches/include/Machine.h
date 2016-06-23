/**
 * \file        Machine.h
 * \class       Machine
 * \brief       classe pour les différentes actions d'un robot concernant une machine
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef MACHINE_H
#define MACHINE_H

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include <common_utils/types.h>
#include <manager_msg/activity.h>
#include <manager_msg/order.h>
#include <deplacement_msg/MoveToPoseAction.h>

#include "FeuClientAction.h"
#include "GripperClientSrv.h"
#include "FinalApproachingClient.h"
#include "ExploInfoSubscriber.h"
#include "NavigationClientAction.h"

using namespace manager_msg;
using namespace final_approach_msg;
using namespace common_utils;

class Machine
{
	protected:

		/* Variables d'instance */
		std::string m_type;
		geometry_msgs::Pose2D m_centerMachine;
		geometry_msgs::Pose2D m_entryMachine;
		geometry_msgs::Pose2D m_exitMachine;
		int zone;
		bool isHere;
		std::string m_name;
		int m_faType;

	public:
		/* Constructeur */
		Machine(int teamColor);

		/* Destructeur */
		virtual ~Machine();

		virtual void FonctionVirtuelle() = 0;

		/* méthodes */
		std::string getType();
		geometry_msgs::Pose2D getCenterMachine();
		geometry_msgs::Pose2D getEntryMachine();
		geometry_msgs::Pose2D getExitMachine();

    void majCenter(geometry_msgs::Pose2D point);
		void majEntry(geometry_msgs::Pose2D point);
		void majExit(geometry_msgs::Pose2D point);

		manager_msg::activity msgToGT(int n_robot, int stateOfOrder, int machine, int n_order);

		void goTo(geometry_msgs::Pose2D pt_dest);
		void take();
		void let();
		void startFinalAp(int8_t machineType, int8_t machineSide, int8_t machineParameter);
		int getFaType(){ return m_faType; }
		const char *getName(){ return m_name.c_str(); }

};

#endif
