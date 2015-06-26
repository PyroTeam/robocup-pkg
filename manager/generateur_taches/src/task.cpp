#include <string>
#include <iostream>
#include <list>
#include <cstdlib>
#include <ros/ros.h>

#include "task.h"

using namespace std;


Task::Task(int intitule,int parametre,int prod,int deb,int fin,int crea,float rat,bool en_traitement, int robot, int fin_exec){
	m_title = intitule;
	m_parameter = parametre;
	m_product = prod;
	m_beginningDelivery = deb;
	m_endDelivery = fin;
	m_creation = crea;
	m_ratio = rat;
	m_inProcess = en_traitement;
	m_robot = robot;
	m_endCarryingOut = fin_exec;
}

int Task::pointPerProduct(){
	int tmp;
	switch(m_product){
		case 0:
			tmp=1;
			break;
		case 1:
			tmp=3;
			break;
		case 2:
			tmp=6;
			break;
		case 3:
			tmp=10;
			break;
	}
	return tmp;
}

bool Task::inTime(double time){
  if((((m_beginningDelivery - time) <= 0) && ((m_endDelivery - time) > 0)) || (time > 14*60))
    return true;
  else
    return false;
}

