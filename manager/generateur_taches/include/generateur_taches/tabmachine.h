#ifndef TABMACHINE_H
#define TABMACHINE_H

#define NBR_MACHINES 6

#include "machine.h"
#include "robot.h"

bool finishedExploration(Machine tabMachine[]);
void updateZone(Machine (&tabMachine)[NBR_MACHINES], Robot tab_robot[]);

#endif
