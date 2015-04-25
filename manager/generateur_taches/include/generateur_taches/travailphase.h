#ifndef TRAVAILPHASE_H
#define TRAVAILPHASE_H

#include "tabmachine.h"
#include "machine.h"
#include "robot.h"
#include "srvorder.h"
#include "travail.h"
#include "ordre.h"
#include "stockage.h"


#include <ros/ros.h>
#include <list> 
#include <iostream>


void travail_phase_exploration(Machine (&tab_machine)[6], Robot (&tabrobot)[3],int &cpt_order, int robot);
void travail_phase_production(std::list<std::list<Tache> > &work, Machine (&tab_machine)[6], Robot (&tabrobot)[3],Stockage (&tab_stock)[6], bool (&take)[3], int &cpt_order, int robot, int &cap_dispo, int &storage,Ordre &order, double temps);
#endif
