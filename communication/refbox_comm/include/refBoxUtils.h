/**
 * \file 		refBoxUtils.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-03-05
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#ifndef REFBOXUTILS_H_
#define REFBOXUTILS_H_


#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "comm_msg/GameState.h"
#include "comm_msg/ExplorationInfo.h"
#include "comm_msg/ExplorationMachine.h"
#include "comm_msg/ReportMachine.h"
#include "comm_msg/MachineInfo.h"
#include "comm_msg/MachineReportInfo.h"
#include "comm_msg/OrderInfo.h"
#include "comm_msg/RobotInfo.h"


#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>
#include <msgs/Team.pb.h>

enum GamePhase
{
    PHASE_PRE_GAME,
    PHASE_SETUP,
    PHASE_EXPLORATION,
    PHASE_PRODUCTION,
    PHASE_POST_GAME
};


comm_msg::GameState llsf2ros_gameState(const llsf_msgs::GameState &llsfGameState, llsf_msgs::Team team_color);
comm_msg::ExplorationInfo llsf2ros_explorationInfo(const llsf_msgs::ExplorationInfo &llsfExplorationInfo, llsf_msgs::Team team_color);
comm_msg::MachineInfo llsf2ros_machineInfo(const llsf_msgs::MachineInfo &llsfMachineInfo);
comm_msg::OrderInfo llsf2ros_orderInfo(const llsf_msgs::OrderInfo &llsfOrderInfo);
comm_msg::RobotInfo llsf2ros_robotInfo(const llsf_msgs::RobotInfo &llsfRobotInfo);


#endif /* REFBOXUTILS_H_ */
