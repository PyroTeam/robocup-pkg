/**
 * \file        GtServerSrv.h
 * \class       GtServerSrv
 * \brief       classe serveur pour le générateur de tâches
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef GTSERVERSRV_H
#define GTSERVERSRV_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include "manager_msg/order.h"
#include "manager_msg/activity.h"
#include "manager_msg/finalApproachingAction.h"
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

		/* Déstructeur */
		virtual  ~GtServerSrv();

		/* Méthodes */
		bool responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res);
		void setId(int id);
		int teamColorOfId(int arTag);
		bool isInput(int arTag);
		int teamColorOfZone(int zone);
		manager_msg::activity getActivityMsg();
		manager_msg::finalApproachingAction getFinalAppAction();
		void interpretationZone();
		void interpretationZone(int zone, zoneCorner_t zoneCorner);
		bool going(geometry_msgs::Pose2D point);
		geometry_msgs::Pose2D calculOutPoint(geometry_msgs::Pose2D pt_actuel, int zone);
		void getSidePoints(int zone, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2);
		bool knownMachineInZone(int zone);
		void getNearestPoint(geometry_msgs::Pose2D &pose
			, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2
			, geometry_msgs::Pose2D **targetPointPtr, geometry_msgs::Pose2D **otherPointPtr);
		void asking(geometry_msgs::Pose2D point);
	private:
		/* Variables d'instance */
		int m_nbrobot;
		int m_color;
		int m_id;
		float m_x;
		float m_y;
		geometry_msgs::Pose2D m_ptTarget;
		manager_msg::activity m_msg;
		std::string m_name;
		manager_msg::finalApproachingAction m_act;
		ExploInfoSubscriber *m_ei;
		LocaSubscriber *m_ls;
    	ros::Publisher m_activity_pub;
    	MyElements m_elements;
};

#endif