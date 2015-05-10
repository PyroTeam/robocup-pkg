#include <list>
#include <iostream>
#include <ros/ros.h>

#include "work.h"
#include "task.h"
#include "tasksList.h"
#include "order.h"

#include "manager_msg/order.h"

using namespace std;
using namespace manager_msg;

// verifie si la demande de la RefBox a déjà prise en compte
bool alreadyInWork(list<list<Task> > work, Order order){
	bool tmp = false;
	list<list<Task> >::iterator workIterator;
	for(workIterator = work.begin(); workIterator != work.end(); workIterator++)
	{
		if( (order.getProduct().getNbrRing() == workIterator->begin()->getProduct()) 
		&& (order.getBeginningDelivery() == workIterator->begin()->getBeginningDelivery()) )
		{
			tmp =true;
		}
	}
	return tmp;
}

// rajoute les nouvelles listes de tâches à faire éxecuter 
void addInWork(list<list<Task> > &work, Order &order,int &availableCap){
	if(!alreadyInWork(work,order) && !order.getProcessed())
	{
		order.setProcessed(true);
		for(int i=0; i< order.getQuantity(); i++)
		{
			if(availableCap > 0)
			{
				work.push_back(creationListTasksProduct(order.getProduct(),order.getBeginningDelivery(), 
														order.getEndDelivery()));
				availableCap --;
			}
			else
			{
				list<list<Task> >::iterator workIterator;
				workIterator = work.begin();
				while(workIterator != work.end() && !uncapInWork(*workIterator))
				{
					workIterator++;
				}
				if(workIterator != work.end())
				{
					list<Task> ltmp = creationListTasksProduct(order.getProduct(),order.getBeginningDelivery(), 
															   order.getEndDelivery());
					int tmpCreation = ltmp.begin()->getCreation();
					workIterator->splice(workIterator->end(), ltmp);
					workIterator->begin()->setCreation(tmpCreation + 30);
				}
			}
		}
	}
}

//verifie s'il y a des taches qui ont un ratio superieur a zero
bool positiveRatio(list<list<Task> > work){
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
list<list<Task> >::iterator maxRatio(list<list<Task> > &work){
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
void ratioCalculus(list<list<Task> > &work,double time,int robot,bool take[]){
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
				t_it->begin()->setRatio(t_it->begin()->pointPerProduct());
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
void getInfoWork(list<list<Task> > work){
	list<list<Task> >::iterator wit;
	int compteur = 1; 
	for(wit = work.begin(); wit != work.end(); wit++)
	{
		cout << "taille de la liste " << compteur << " = " << wit->size() <<
			    " | ratio = "<< wit->begin()->getRatio() <<
    			" | intitule première tâche = " << wit->begin()->getTitle()  << 
      			" " << wit->begin()->getParameter() <<  
      			" debut_livr : " << wit->begin()->getBeginningDelivery() <<
      			" fin_livr : " << wit->begin()->getEndDelivery() << endl;
		compteur++;
	}
}			
			
//enlève ce qui est déjà fait
void cleanWork(list<list<Task> > &work,list<list<Task> >::iterator &it,double time){
	if(!it->empty()) 
	{
		it->pop_front();
		it->begin()->setEndCarryingOut(it->begin()->getCreation() + time);
	}
	if(it->empty()) 
	{
		work.erase(it);
	}
}

//quelques traitements à faire en plus en cas de tâche particulière
void particularTasksInWork(list<list<Task> > ::iterator &it, int &availableCap, int &storage,double time){
	//si la seule tâche est de décapsuler dans la liste
	if((it->begin()->getTitle() == int(orderRequest::UNCAP)) && (it->size() == 1))
	{ 
		availableCap ++;
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
void finishedTasks(list<list<Task> > &work, int robot, double time){
	list<list<Task> >::iterator wit;
	for(wit=work.begin(); wit!=work.end(); wit++)
	{
		if(wit->begin()->getEndCarryingOut() > time)
		{
			wit->begin()->setInProcess(false);
		}
	}
}  
  
