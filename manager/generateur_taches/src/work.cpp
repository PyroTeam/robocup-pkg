#include <list>
#include <iostream>
#include <ros/ros.h>

#include "work.h"
#include "task.h"
#include "tasksList.h"


#include "manager_msg/order.h"

using namespace std;
using namespace manager_msg;

// verifie si la demande de la RefBox a déjà prise en compte
bool alreadyInWork(list<list<Task> > work, comm_msg::Order order)
{
	bool tmp = false;
	list<list<Task> >::iterator workIterator;
	for(workIterator = work.begin(); workIterator != work.end(); workIterator++)
	{
		if((order.delivery_period_end == workIterator->begin()->getEndDelivery()) 
		  && (order.delivery_period_begin == workIterator->begin()->getBeginningDelivery()))
		{
			tmp =true;
		}
	}
	return tmp;
}

// rajoute les nouvelles listes de tâches à faire éxecuter 
void addInWork(list<list<Task> > &work, std::vector<comm_msg::Order> tabOrders,int (&availableCap)[2], std::vector<bool> &ordersInProcess)
{
	if(!tabOrders.empty() )
	{
		for(int j=0; j<tabOrders.size(); j++)
		{
			if(!ordersInProcess[j] && !alreadyInWork(work,tabOrders[j]))
			{
				ordersInProcess[j]=true;
				for(int i=0; i< tabOrders[j].quantity_requested; i++)
				{
					addAccordingToCap(work, tabOrders[j], availableCap);
				}
			}
		}
	}
}

//verifie s'il y a des taches qui ont un ratio superieur a zero
bool positiveRatio(list<list<Task> > work)
{
	bool tmp = false;
	list<list<Task> >::iterator it;
	for(it = work.begin(); it != work.end(); it++)
	{
		if(it->begin()->getRatio() > 0)
		{
			tmp = true;
		}
	}
	return tmp;
}

//renvoie la tâche qui le plus grand ratio
list<list<Task> >::iterator maxRatio(list<list<Task> > &work)
{
	list<list<Task> >::iterator workIterator;
	list<list<Task> >::iterator tmp = work.begin();
	for(workIterator = work.begin(); workIterator != work.end(); workIterator++)
	{
		if(workIterator->begin()->getRatio() > tmp->begin()->getRatio())
		{
			tmp = workIterator;
		}
	}
	return tmp;
}

//calcule le ratio de chaque première tâche de chaque liste de tâche
void ratioCalculus(list<list<Task> > &work,double time,int robot,bool take[])
{
	list<list<Task> >::iterator t_it;
	for(t_it = work.begin(); t_it != work.end(); t_it++)
	{
		t_it->begin()->setRatio(0);
		if(t_it->begin()->getTitle() == int(orderRequest::DESTOCK))
		{
			if(t_it->begin()->inTime(time))
			{
				t_it->begin()->setRatio(100);
			}
		}
		else
		{
			if((t_it->size() == 1) && (t_it->begin()->getTitle() == int(orderRequest::UNCAP)))
			{
				t_it->begin()->setRatio(0.1);
			}
			else
			{
				t_it->begin()->setRatio(t_it->begin()->pointPerComplexity());
			}
		}
		if(take[robot]==true)
		{
			if(t_it->begin()->getTitle() == int(orderRequest::PUT_CAP) ||
			   t_it->begin()->getTitle() == int(orderRequest::PUT_RING) ||
			   t_it->begin()->getTitle() == int(orderRequest::DESTOCK))
			{
				t_it->begin()->setRatio(300);
			}
		}
		if(t_it->begin()->getInProcess())
		{
			t_it->begin()->setRatio(0);
		}
		t_it->begin()->setRatio(t_it->begin()->getRatio() / t_it->begin()->getCreation());
	}
}

//affiche des infos sur la structure contenant les tâches à réaliser
void getInfoWork(list<list<Task> > work)
{
	list<list<Task> >::iterator wit;
	int compteur = 1;
	for(wit = work.begin(); wit != work.end(); wit++)
	{
		ROS_INFO("Taille de la liste %d : %d ratio: %f intitule premiere tache: %d parametre: %d debut_livr: %d fin_livr: %d",
		         compteur,(int)wit->size(),(float)wit->begin()->getRatio(),wit->begin()->getTitle(),
		         wit->begin()->getParameter(),wit->begin()->getBeginningDelivery(),wit->begin()->getEndDelivery());
		compteur++;
	}
}

//enlève ce qui est déjà fait
void cleanWork(list<list<Task> > &work,list<list<Task> >::iterator &it,double time){
	if(!it->empty())
	{
		it->pop_front();
		//it->begin()->setTaskEnd(it->begin()->getTaskEnd() + time);
	}
	if(it->empty())
	{
		work.erase(it);
	}
}

//quelques traitements à faire en plus en cas de tâche particulière
void particularTasksInWork(list<list<Task> > ::iterator &it, int (&availableCap)[2], int &storage,double time){
	//si la seule tâche est de décapsuler dans la liste
	if((it->begin()->getTitle() == int(orderRequest::UNCAP)) && (it->size() == 1))
	{
		switch(it->begin()->getParameter())
		{
			case int(orderRequest::BLACK) :
				availableCap[0] ++;
				break;
			case int(orderRequest::GREY) :
				availableCap[1] ++;
				break;
		}
		storage ++;
	}
	//si on a un product fini
	if(it->begin()->getTitle() == int(orderRequest::DELIVER))
	{
		if(it->begin()->inTime(time))
		{
			it->begin()->setParameter(orderRequest::DS);
		}
		else
		{
			it->begin()->setParameter(int(orderRequest::STOCK));
		}
	}
}

//verifie les taches terminees
void finishedTasks(list<list<Task> > &work, double time){
	list<list<Task> >::iterator wit;
	for(wit=work.begin(); wit!=work.end(); wit++)
	{
		if(wit->begin()->getTaskEnd() >= time)
		{
			wit->begin()->setInProcess(false);
			cleanWork(work,wit,time);
		}
	}
}

void addNewTasksList(std::list<std::list<Task> > &work, comm_msg::Order order, int availableCap)
{
	if(availableCap > 0)
	{
		Product product(order.complexity,order.base_color,order.ring_colors,order.cap_color);
		work.push_back(creationListTasksProduct(product,order.delivery_period_begin, order.delivery_period_end));
		availableCap --;
	}
	else
	{
		Product product(order.complexity,order.base_color,order.ring_colors,order.cap_color);
		list<Task> latmp = creationListTasksAction(orderRequest::UNCAP,product,order.delivery_period_begin, order.delivery_period_end);
		list<Task> lptmp = creationListTasksProduct(product,order.delivery_period_begin, order.delivery_period_end);
		int tmpCreation = lptmp.begin()->getCreation();
		latmp.splice(latmp.end(), lptmp);
		latmp.begin()->setCreation(tmpCreation + 30);
		work.push_back(latmp);
	}
}

void addAccordingToCap(std::list<std::list<Task> > &work, comm_msg::Order order,int (&availableCap)[2])
{
	if(order.cap_color == comm_msg::Order::CAP_BLACK)
	{
		addNewTasksList(work, order, availableCap[0]);
	}
	if(order.cap_color == comm_msg::Order::CAP_GREY)
	{
		addNewTasksList(work, order, availableCap[1]);
	}
}
