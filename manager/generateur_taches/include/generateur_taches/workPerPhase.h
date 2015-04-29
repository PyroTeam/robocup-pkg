#ifndef WORKPERPHASE_H
#define WORKPERPHASE_H

#include "tabmachine.h"
#include "machine.h"
#include "robot.h"
#include "srvorder.h"
#include "work.h"
#include "order.h"
#include "storage.h"
#include "correspondanceZE.h"

#include <ros/ros.h>
#include <list>
#include <iostream>


void workInExplorationPhase(Machine (&tabMachine)[6], Robot (&tabRobot)[3],int &cptOrder, int robot ,int &cptZone, 								CorrespondanceZE &correspondanceZE);

void workInProductionPhase(std::list<std::list<Task> > &work, Machine (&tabMachine)[6], Robot (&tabRobot)[3],
						   Storage (&tabStock)[6], bool (&take)[3], int &cptOrder, int robot, int &availableCap, 
						   int &storage,Order &order, double time);
#endif
