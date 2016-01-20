/**
 * \file 		main_node.cpp
 * \brief		Programme principal du générateur de tâches
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-30
 * \copyright	PyroTeam, Polytech-Lille
 */

#include <list>
#include <iostream>
#include <ros/ros.h>

#include "storage.h"
#include "action.h"
#include "task.h"
#include "tasksList.h"
#include "manager_msg/order.h"
#include "work.h"
#include "robot.h"
#include "product.h"
#include "machine.h"
#include "workPerPhase.h"
#include "gameState.h"
#include "correspondanceZE.h"
#include "srvorder.h"
#include "orderInfo.h"
#include "robotInfo.h"

#include "comm_msg/Order.h"
#include "comm_msg/Robot.h"
#include "comm_msg/GameState.h" 

using namespace std;
using namespace manager_msg;

int main(int argc, char **argv) 
{

	/*** INITIALISATION ***/
	ros::init(argc, argv, "action_node");
	int teamColor = 0;
	Action action;
	GameState gameState;
	ros::Rate loop_rate(1);
	int storage =0, id=0, cptOrder = 0, k=0;
	int availableCap[2] = {0,0};
	Storage tabStock[6];
	Robot tabRobot[3];
	Product act(0,0,{},0);
	list< list<Task> > work;
	for(int i=0;i<6;i++)
	{
		work.push_back(creationListTasksAction(7,act,0,0));
	}
	getInfoWork(work);
	Machine tabMachine[6];
	bool take[3] = {false,false,false};
	double t0 = ros::Time::now().toSec();
	double time = t0;
	int cptZone = 0;
	CorrespondanceZE correspondanceZE;
	OrderInfo orderInfo;
	std::vector<comm_msg::Order> tabOrders;
	std::vector<bool> ordersInProcess;
	RobotInfo robotInfo;
	std::vector<comm_msg::Robot> tabRobotInfo;
	bool noProblem = true;
	for(int i=0;i<20;i++)
	{
		ordersInProcess.push_back(false);
	}
	/***FONCTION PRINCIPALE ***/
	
	while(ros::ok()) 
	{
	// il y a trois robots
		for(int j=0; j<3; j++)
		{
			time = ros::Time::now().toSec() - t0;
			ROS_INFO("temps en sec = %d",(int)time);
			action.updateRobot(tabRobot);
			tabRobotInfo = robotInfo.getRobots();
			noProblem = robotState(tabRobotInfo,teamColor,j,tabRobot);
			//mettre a jour les infos envoyees par la refbox
			ROS_INFO("Busy :%d | noProblem :%d", tabRobot[j].getBusy(), noProblem);
			if(!tabRobot[j].getBusy() && noProblem) 
			{
				ROS_INFO("Not busy");
				if(gameState.getPhase() == comm_msg::GameState::EXPLORATION && cptZone<12)
				{
					workInExplorationPhase(tabMachine,tabRobot,cptOrder,j,cptZone, correspondanceZE);
				}
				if(gameState.getPhase() == comm_msg::GameState::PRODUCTION && !work.empty())
				{
					tabOrders = orderInfo.getOrders();
					workInProductionPhase(work, tabMachine, tabRobot, tabStock, take, cptOrder, j, availableCap, storage, tabOrders, time, ordersInProcess);
					getInfoWork(work);
				}
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	return 0;
}
