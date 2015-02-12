/**
 * \file 		receiveHandler.h
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-12
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */
 
#ifndef RECEIVEHANDLER_H_
#define RECEIVEHANDLER_H_

#include <stdio.h>
#include <iostream>

#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>
#include <msgs/Team.pb.h>


typedef google::protobuf::Message protoMsg;

bool fireBeaconSignal(protoMsg &m);
bool fireGameState(protoMsg &m);
bool fireOrderInfo(protoMsg &m);
bool fireVersionInfo(protoMsg &m);
bool fireExplorationInfo(protoMsg &m);
bool fireMachineInfo(protoMsg &m);
bool fireMachineReportInfo(protoMsg &m);
bool fireRobotInfo(protoMsg &m);
bool fireUnknown(protoMsg &m);


#endif /* RECEIVEHANDLER_H_ */
