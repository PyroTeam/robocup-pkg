/**
 * \file 		workPerPhase.h
 * \brief		fonctions pour déterminer ce qu'il y a à faire en phase d'exploration et celle de production
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef WORKPERPHASE_H
#define WORKPERPHASE_H

#include "tabmachine.h"
#include "machine.h"
#include "robot.h"
#include "srvorder.h"
#include "work.h"
#include "storage.h"
#include "correspondanceZE.h"

#include "comm_msg/Order.h"

#include <ros/ros.h>
#include <list>
#include <vector>
#include <iostream>

/**
 *	\brief		ce qu'il y a à faire en phase d'exploration
 */
void workInExplorationPhase(Machine (&tabMachine)[6], Robot (&tabRobot)[3],int &cptOrder, int robot ,int &cptZone,
							CorrespondanceZE &correspondanceZE, common_utils::RobotPoseSubscriber &poseSub);

/**
 *	\brief		ce qu'il y a à faire en phase de production
 */
void workInProductionPhase(std::list<std::list<Task> > &work, Machine (&tabMachine)[6], Robot (&tabRobot)[3],
						   Storage (&tabStock)[6], bool (&take)[3], int &cptOrder, int robot, int (&availableCap)[2],
						   int &storage,std::vector<comm_msg::Order> &tabOrders, double time, std::vector<bool> &ordersInProcess);
#endif
