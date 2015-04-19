/**
 * \file 		refBoxUtils.h
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-03-05
 * \copyright	PyroTeam, Polytech-Lille
 * \license
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


comm_msg::GameState llsf2ros_gameState(llsf_msgs::GameState llsfGameState, llsf_msgs::Team team_color);
comm_msg::ExplorationInfo llsf2ros_explorationInfo(llsf_msgs::ExplorationInfo llsfExplorationInfo, llsf_msgs::Team team_color);

#endif /* REFBOXUTILS_H_ */

