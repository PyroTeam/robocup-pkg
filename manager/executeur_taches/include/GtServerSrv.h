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

#define C_CS1_IN   1
#define C_CS1_OUT  2
#define C_CS2_IN   17
#define C_CS2_OUT  18
#define C_RS1_IN   33
#define C_RS1_OUT  34
#define C_RS2_IN   177
#define C_RS2_OUT  178
#define C_BS_IN    65
#define C_BS_OUT   66
#define C_DS_IN    81
#define C_DS_OUT   82

#define M_CS1_IN   97
#define M_CS1_OUT  98
#define M_CS2_IN   113
#define M_CS2_OUT  114
#define M_RS1_IN   129
#define M_RS1_OUT  130
#define M_RS2_IN   145
#define M_RS2_OUT  146
#define M_BS_IN    161
#define M_BS_OUT   162
#define M_DS_IN    49
#define M_DS_OUT   50

#define CYAN 0
#define MAGENTA 1

class GtServerSrv
{
	public:
		/* Constructeur */
		GtServerSrv();

		/* Déstructeur */
		virtual  ~GtServerSrv();

		/* Méthodes */
		bool responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res);
		void setId(int id);
		int teamColorOfId(int arTag);
		int teamColorOfZone(int zone);
		manager_msg::activity getActivityMsg();
		final_approach_msg::FinalApproachingAction getFinalAppAction();
		void interpretationZone();
		void going(geometry_msgs::Pose2D point);
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
		manager_msg::activity m_msg;
		std::string m_name;
		final_approach_msg::FinalApproachingAction m_act;
		ExploInfoSubscriber *m_ei;
		LocaSubscriber *m_ls;
    	ros::Publisher m_activity_pub;
};

#endif