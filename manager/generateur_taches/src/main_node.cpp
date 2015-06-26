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

#include "comm_msg/GameState.h" 

using namespace std;
using namespace manager_msg;

int main(int argc, char **argv) 
{

	/*** INITIALISATION ***/
	ros::init(argc, argv, "action_node");
	Action action;
	GameState gameState;
	ros::Rate loop_rate(1);
	int availableCap = 0,storage =0, id=0, cptOrder = 0, k=0;
	Storage tabStock[6];
	Robot tabRobot[3];
	vector<int> black(1,10);
	Product act(0,black);
	list< list<Task> > work;
	for(int i=0;i<6;i++)
	{
		work.push_back(creationListTasksAction(7,act,0,0));
	}
	getInfoWork(work);
	cout <<""<<endl;
	vector<int> couleurs;
	couleurs.push_back(10);
	couleurs.push_back(20);
	Product product(0,couleurs);
	Order order(product,20,30,2,false);
	Machine tabMachine[6];
	bool take[3] = {false,false,false};
	double t0 = ros::Time::now().toSec();
	double time = t0;
	int cptZone = 0;
	CorrespondanceZE correspondanceZE;



	/***FONCTION PRINCIPALE ***/
	
	while(ros::ok()) 
	{
	// il y a trois robots
		for(int j=0; j<3; j++)
		{
			time = ros::Time::now().toSec() - t0;
			cout << "time en sec = " << time << endl;
			action.updateRobot(tabRobot);
			std::cout << "etat de tabRobot["<<j<<"] : " << tabRobot[j].getBusy() << std::endl;
			//mettre a jour les infos envoyees par la refbox
			if(!tabRobot[j].getBusy() && cptZone<12)
			{
				if(gameState.getPhase() == comm_msg::GameState::EXPLORATION)
				{
					workInExplorationPhase(tabMachine,tabRobot,cptOrder,j,cptZone, correspondanceZE);
				}
				if(gameState.getPhase() == comm_msg::GameState::PRODUCTION && !work.empty()){
					workInProductionPhase(work, tabMachine, tabRobot, tabStock, take, cptOrder, j, availableCap, storage, order, time);
					getInfoWork(work);
				}
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	return 0;
}
