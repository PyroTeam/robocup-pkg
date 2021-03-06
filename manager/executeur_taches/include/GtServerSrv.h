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
#include "common_utils/RobotPoseSubscriber.h"
#include "geometry_utils/geometry_utils.h"

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
    GtServerSrv(int teamColor);
    virtual  ~GtServerSrv();

    bool responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res);
    void setId(int id);
    bool isInput(int arTag);
    int teamColorOfZone(int zone);
    manager_msg::activity getActivityMsg();
    final_approach_msg::FinalApproachingAction getFinalAppAction();
    void interpretationZone(int zone, zoneCorner_t zoneCorner);
    bool going(const geometry_msgs::Pose2D &point, size_t nbAttempt = 0);
    void getSidePoints(int zone, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2);
    bool knownMachineInZone(int zone);

    void msgToGT(int stateOfOrder, int machine, int n_order);

private:
    ros::NodeHandle m_nh;

    int m_nbrobot;
    int m_color;
    int m_id;

    geometry_msgs::Pose2D m_explo_target;

    manager_msg::activity m_msg;
    std::string m_name;
    final_approach_msg::FinalApproachingAction m_act;
    ExploInfoSubscriber *m_ei;
    LocaSubscriber *m_ls;
    ros::Publisher m_activity_pub;
    MyElements m_elements;

    common_utils::RobotPoseSubscriber m_poseSub;
};

#endif
