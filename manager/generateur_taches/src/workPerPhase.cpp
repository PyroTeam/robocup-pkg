#include "workPerPhase.h"
#include "tabmachine.h"
#include "machine.h"
#include "robot.h"
#include "srvorder.h"
#include "work.h"
#include "storage.h"
#include "storageChart.h"
#include "product.h"
#include "tasksList.h"
#include "correspondanceZE.h"
#include "orderInfo.h"

#include <ros/ros.h>
#include <list>
#include <iostream>

#include "comm_msg/Order.h"
#include "manager_msg/order.h"

using namespace manager_msg;
using namespace std;

void workInExplorationPhase(Machine (&tabMachine)[6], Robot (&tabRobot)[3],int &cptOrder, int robot ,int &cptZone, 
							CorrespondanceZE &correspondanceZE){
	//update_zone(tabMachine,tabRobot); //trouver une maniere efficace d'attribuer aux machines un robot
	int cptMachine=0;
	vector<int> zone = correspondanceZE.getUsefulZone();
	if(zone.size() == 12)
	{
		Srvorder srvexplo(ros::Time::now(),cptOrder,robot,orderRequest::DISCOVER,orderRequest::NONE,zone[cptZone]);
		ROS_INFO("Robot n° %d execute la tache DISCOVER sur la zone n %d",robot,zone[cptZone]);
		cptZone++;
		cptOrder++;
		if(srvexplo.getAccepted())
		{
		cptMachine++;
		tabRobot[robot].setBusy(true);
		}
	}
}


void workInProductionPhase(std::list<std::list<Task> > &work, Machine (&tabMachine)[6], Robot (&tabRobot)[3], 
						   Storage (&tabStock)[6], bool (&take)[3], int &cptOrder, int robot, int (&availableCap)[2], 
						   int &storage,std::vector<comm_msg::Order> &tabOrders, double time, std::vector<bool> &ordersInProcess)
{
	if(!tabRobot[robot].getBusy()) 
	{
		int id = 0;
		addInWork(work,tabOrders,availableCap, ordersInProcess);
		//s'il y a au moins un ratio strictement positif
		if(positiveRatio(work))
		{
			list<list<Task> >::iterator wit;
			//boucle permettant de déterminer s'il faut rajouter du temps de traitement de machine
			for(wit = work.begin(); wit != work.end(); wit++)
			{
				if(wit->begin()->getRobot() == robot)
				{
					wit->begin()->setTaskEnd(time + wit->begin()->getMachineTime());
					wit->begin()->setRobot(0);
				}
			}
			finishedTasks(work,time);
			//partie permettant de déterminer la tâche la plus prioritaire
			ratioCalculus(work,time,robot,take);
			list<list<Task> >::iterator it = maxRatio(work);
			particularTasksInWork(it,availableCap,storage,time);
			//partie permettant de retrouver l'id de l'objet qui était stocké
			if(it->begin()->getTitle() == orderRequest::DESTOCK)
			{
				id=findId(tabStock,it->begin()->getBeginningDelivery(), it->begin()->getEndDelivery());
			}
			//service envoyé à l'exécuteur de tâches
			Srvorder srv(ros::Time::now(),cptOrder,robot+1,it->begin()->getTitle(),it->begin()->getParameter(),0);
			ROS_INFO("Robot: %d tâche: %d parametre: %d id: %d",robot+1,it->begin()->getTitle(),
			         it->begin()->getParameter(),srv.getId());
			cptOrder++;
			//dans le cas où c'est accepté
			if(srv.getAccepted())
			{
				it->begin()->setInProcess(true);
				it->begin()->setRobot(robot);
				//s'il faut stocker un produit
				if((it->begin()->getTitle() == orderRequest::DELIVER) && 
				   (it->begin()->getParameter() == orderRequest::STOCK))
				{
					vector<uint8_t> nothing(1,1);
					//produit bidon servant juste pour creationListTasksAction
					Product prod_tmp(it->begin()->getComplexity(),1,nothing,1);
					list<Task> ltmp = creationListTasksAction(int(orderRequest::DESTOCK),prod_tmp,
									  it->begin()->getBeginningDelivery(),it->begin()->getEndDelivery());
					it->splice(it->end(),ltmp);
					Storage stock(it->begin()->getComplexity(),it->begin()->getBeginningDelivery(),
								  it->begin()->getEndDelivery());
					tabStock[srv.getId()]=stock;
				}
				//s'il faut déstocker un produit
				if(it->begin()->getTitle() == orderRequest::DESTOCK)
				{
					tabStock[id].setBeginningDelivery(0);
					tabStock[id].setEndDelivery(0);
				}
				//si le robot doit chercher un produit (fini ou non) auprès d'une machine
				if((it->begin()->getTitle() == orderRequest::TAKE_BASE) 
				    || (it->begin()->getTitle() == orderRequest::TAKE_CAP) 
				    || (it->begin()->getTitle() == orderRequest::TAKE_CAP))
				{
					take[robot] = true;
				}
				else
				{
					take[robot] = false;
				}
				//mettre à jour le tableau contenant les infos de l'ensemble des robots
				tabRobot[robot].setBusy(true);
			}
		}
		//order.setQuantity(0);
		id=0;
	}
}
