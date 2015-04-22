#ifndef TABMACHINE_H
#define TABMACHINE_H

#define NBR_MACHINES 6

#include "machine.h"
#include "robot.h"

bool exploration_finie(Machine tab_machine[]);
void update_zone(Machine (&tab_machine)[NBR_MACHINES], Robot tab_robot[]);

#endif
