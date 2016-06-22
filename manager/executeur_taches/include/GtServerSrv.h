/**
 * \file        GtServerSrv.h
 * \class       GtServerSrv
 * \brief       classe serveur pour le générateur de tâches
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef GTSERVERSRV_H
#define GTSERVERSRV_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include "manager_msg/order.h"
#include "manager_msg/activity.h"
#include "final_approach_msg/FinalApproachingAction.h"
#include "common_utils/types.h"

#include "ExploInfoSubscriber.h"
#include "Machine.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"
#include "BaseStation.h"
#include "MyElements.h"
#include "ArTagClientSrv.h"
#include "ReportingMachineSrvClient.h"
#include "LocaSubscriber.h"
#include "RobotPoseSubscriber.h"
#include "FinalApproachingClient.h"

enum zoneCorner_t
{
	BOTTOM_LEFT,
	BOTTOM_RIGHT,
	TOP_LEFT,
	TOP_RIGHT
};

class GtServerSrv
{
	public:
		/* Constructeur */
		GtServerSrv(int teamColor);

		/* Destructeur */
		virtual  ~GtServerSrv();

		/* Méthodes */
		bool responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res);
		void setId(int id);
		bool isInput(int arTag);
		int teamColorOfZone(int zone);
		manager_msg::activity getActivityMsg();
		final_approach_msg::FinalApproachingAction getFinalAppAction();
		void interpretationZone(int zone, zoneCorner_t zoneCorner);
		bool going(geometry_msgs::Pose2D point);
		geometry_msgs::Pose2D calculOutPoint(geometry_msgs::Pose2D pt_actuel, int zone);
		void getSidePoints(int zone, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2);
		bool knownMachineInZone(int zone);
		void getNearestPoint(geometry_msgs::Pose2D &pose
			, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2
			, geometry_msgs::Pose2D **targetPointPtr, geometry_msgs::Pose2D **otherPointPtr);

	private:
		/* Variables d'instance */
		int m_nbrobot;
		int m_color;
		int m_id;

		geometry_msgs::Pose2D m_explo_target;

		manager_msg::activity m_msg;
		std::string m_name;
		final_approach_msg::FinalApproachingAction m_act;
		ExploInfoSubscriber *m_ei;
		LocaSubscriber *m_ls;
    RobotPoseSubscriber *m_rp;
    ros::Publisher m_activity_pub;
    MyElements m_elements;
};

#endif
