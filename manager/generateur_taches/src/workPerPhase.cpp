#include "workPerPhase.h"
#include "tabmachine.h"
#include "machine.h"
#include "robot.h"
#include "srvorder.h"
#include "work.h"
#include "order.h"
#include "storage.h"
#include "storageChart.h"
#include "product.h"
#include "tasksList.h"
#include "correspondanceZE.h"

#include <ros/ros.h>
#include <list>
#include <iostream>

#include "manager_msg/order.h"

using namespace manager_msg;
using namespace std;

void workInExplorationPhase(Machine (&tabMachine)[6], Robot (&tabRobot)[3],int &cptOrder, int robot ,int &cptZone, 								CorrespondanceZE &correspondanceZE){
	//update_zone(tabMachine,tabRobot); //trouver une maniere efficace d'attribuer aux machines un robot
	int cptMachine=0;
	vector<int> zone = correspondanceZE.getUsefulZone();
	std::cout << "Taile zone : " << zone.size() << std::endl;
	if(zone.size() == 12)
	{
		Srvorder srvexplo(ros::Time::now(),cptOrder,robot,orderRequest::DISCOVER,orderRequest::NONE,zone[cptZone]);
		cout <<"Robot n°"<<robot<<" execute la tache DISCOVER sur la zone n "<<zone[cptZone]<<endl;
		cptZone++;
		cptOrder++;
		if(srvexplo.getAccepted())
		{
		cptMachine++;
		tabRobot[robot].setOccuped(true);
		}
	}
}


void workInProductionPhase(std::list<std::list<Task> > &work, Machine (&tabMachine)[6], Robot (&tabRobot)[3], 
						   Storage (&tabStock)[6], bool (&take)[3], int &cptOrder, int robot, int &availableCap, 
						   int &storage,Order &order, double time)
{
	if(!tabRobot[robot].getOccuped())
	{
		int id = 0;
		finishedTasks(work,robot,time);
		if(positiveRatio(work))
		{
			ratioCalculus(work,time,robot,take);
			list<list<Task> >::iterator it = maxRatio(work);
			particularTasksInWork(it,availableCap,storage,time);
			if(it->begin()->getTitle() == orderRequest::DESTOCK)
			{
				id=findId(tabStock,it->begin()->getBeginningDelivery(), it->begin()->getEndDelivery());
			}
			Srvorder srv(ros::Time::now(),cptOrder,robot+1,it->begin()->getTitle(),it->begin()->getParameter(),0);
			cout <<"Robot n°"<<robot+1<<" tâche: "<<it->begin()->getTitle()<<" parametre: "
				 <<it->begin()->getParameter() <<" id: " << srv.getId()<<"\n"<<endl;
			cptOrder++;
			if(srv.getAccepted())
			{
				it->begin()->setInProcess(true);
				it->begin()->setRobot(robot);
				if((it->begin()->getTitle() == orderRequest::DELIVER) && 
				   (it->begin()->getParameter() == orderRequest::STOCK))
				{
					vector<int> nothing(1,20);
					Product prod_tmp(it->begin()->getProduct(),nothing);
					list<Task> ltmp = creationListTasksAction(int(orderRequest::DESTOCK),prod_tmp,
									  it->begin()->getBeginningDelivery(),it->begin()->getEndDelivery());
					it->splice(it->end(),ltmp);
					Storage stock(it->begin()->getProduct(),it->begin()->getBeginningDelivery(),
								  it->begin()->getEndDelivery(),srv.getId());
				}
				if((it->begin()->getTitle() == 0) || (it->begin()->getTitle() == 2) || (it->begin()->getTitle() == 4))
				{
					take[robot] = true;
				}
				else
				{
					take[robot] = false;
				}
				cleanWork(work,it,time);
				tabRobot[robot].setOccuped(true);
			}
		}
		//on prend en compte les ordres de la refbox
		addInWork(work,order,availableCap);
		order.setQuantity(0);
		id=0;
	}
}
