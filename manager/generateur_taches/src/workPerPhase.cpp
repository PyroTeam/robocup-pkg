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
							CorrespondanceZE &correspondanceZE, 	common_utils::RobotPoseSubscriber &poseSub){
	//update_zone(tabMachine,tabRobot); //trouver une maniere efficace d'attribuer aux machines un robot
	#if 0
	int cptMachine=0;
	vector<int> zone = correspondanceZE.getUsefulZone();
	ROS_INFO("In workInExplorationPhase");
	if(zone.size() == 12)
	{
		Srvorder srvexplo(ros::Time::now(),cptOrder,robot,orderRequest::DISCOVER,orderRequest::NONE,zone[cptZone]);
		ROS_INFO("Robot %d execute la tache DISCOVER sur la zone %d",robot,zone[cptZone]);
		cptZone++;
		cptOrder++;
		if(srvexplo.getAccepted())
		{
			cptMachine++;
			tabRobot[robot].setBusy(true);
		}
	}
	#endif

  ROS_INFO("taille de m_exploredZones = %d", int(correspondanceZE.m_exploredZones.size()));

	// if(int(correspondanceZE.m_exploredZones.size()) < 6)
  if(int(correspondanceZE.m_notExploredZones.size()) == 0 && int(correspondanceZE.m_unkownZones.size()) == 6)
  {
    ROS_INFO("I've done my job in the exploration phase :D ");
  }
	else{
			int cptMachine=0;
			int zone = correspondanceZE.getBestZone(poseSub);
	    bool foundInExplored = correspondanceZE.m_locaSub.foundInExplored(zone);

			if(zone == -1 )
			{
				ROS_ERROR("HELP ME! getBestZone() has returned -1, what's wrong?");
	      return;
			}
	    else if(foundInExplored)
	    {
	      ROS_ERROR("HELP ME! zone %d is already explored",zone);
	      return;
	    }

	    ROS_INFO("----> In workInExplorationPhase <----");

	    ROS_INFO("getBestZone() returned %d", zone);

			Srvorder srvexplo(ros::Time::now(),cptOrder,robot,orderRequest::DISCOVER,orderRequest::NONE,zone);
			ROS_INFO("Robot %d execute la tache DISCOVER sur la zone %d",robot,zone);
			cptZone++;
			cptOrder++;

#define USE_QUICK_AND_DIRTY_SOLUTION

// Retirera la machine des machines à explorer, en cas de succès ou d'echec définitif
// Un échec définitif est un echec sans demande de renvoi d'ordre pour cette zone : 'NeedToResendOrder'
// NB: Le problème du NeedToResendOrder actuel c'est qu'il renvoit l'ordre tout de suite après l'echec
#ifdef USE_ORIGINAL_SOLUTION
			// Executeur a accepté l'ordre et ca s'est bien passé OU Executeur a refusé l'ordre et pas besoin de re-explorer la zone
			if(srvexplo.getAccepted() || (!srvexplo.getAccepted() && !srvexplo.getNeedToResendOrder()) )
			{
					if(srvexplo.getAccepted())
					{
						cptMachine++;
						tabRobot[robot].setBusy(true);
					}

					bool foundInUnkown = correspondanceZE.m_locaSub.foundInUnkown(zone);
					bool foundInNotExplored =  correspondanceZE.m_locaSub.foundInNotExplored(zone);

					if(foundInUnkown)
					{
						correspondanceZE.m_locaSub.removeFromUnkown(zone);
					}
					else if(foundInNotExplored)
					{
						correspondanceZE.m_locaSub.removeFromNotExplored(zone);
					}

					else if(foundInUnkown && foundInNotExplored)
					{
						ROS_WARN("/!\\ WARNING : zone %d was founded in m_unkownZones & m_notExploredZones ", zone);
					}

					ROS_INFO("foundInUnkown = %d, foundInNotExplored = %d", foundInUnkown, foundInNotExplored);
					ROS_INFO("unkownSize = %d, notExploredSize = %d, exploredSize = %d ", correspondanceZE.m_locaSub.m_unkownZones.size(), correspondanceZE.m_locaSub.m_notExploredZones.size(), correspondanceZE.m_locaSub.m_exploredZones.size());

					correspondanceZE.m_locaSub.pushToExploredList(zone);

					ROS_INFO("----> Done exploring this zone <----");
			 }

			 // Executeur a refusé l'ordre et besoin de re-explorer la zone
			 else if(!srvexplo.getAccepted() && srvexplo.getNeedToResendOrder())
			 {

			 }
#endif

// Retirera la machine des machines à explorer quelque soit le résultat du service
#ifdef USE_QUICK_AND_DIRTY_SOLUTION
			if(srvexplo.getAccepted())
			{
				cptMachine++;
				tabRobot[robot].setBusy(true);
			}

			bool foundInUnkown = correspondanceZE.m_locaSub.foundInUnkown(zone);
			bool foundInNotExplored =  correspondanceZE.m_locaSub.foundInNotExplored(zone);

			if(foundInUnkown)
			{
				correspondanceZE.m_locaSub.removeFromUnkown(zone);
			}
			else if(foundInNotExplored)
			{
				correspondanceZE.m_locaSub.removeFromNotExplored(zone);
			}

			else if(foundInUnkown && foundInNotExplored)
			{
				ROS_ERROR("/!\\ WARNING : zone %d was founded in m_unkownZones & m_notExploredZones ", zone);
			}

			ROS_INFO("foundInUnkown = %d, foundInNotExplored = %d", foundInUnkown, foundInNotExplored);
			ROS_INFO("unkownSize = %d, notExploredSize = %d, exploredSize = %d ", correspondanceZE.m_locaSub.m_unkownZones.size(), correspondanceZE.m_locaSub.m_notExploredZones.size(), correspondanceZE.m_locaSub.m_exploredZones.size());

			correspondanceZE.m_locaSub.pushToExploredList(zone);

			ROS_INFO("----> Done exploring this zone <----");
#endif

			 ROS_INFO("accepted = %d needToResendOrder = %d", srvexplo.getAccepted(), srvexplo.getNeedToResendOrder());
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
