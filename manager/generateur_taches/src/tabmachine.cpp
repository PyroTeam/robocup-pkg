#include "tabmachine.h"
#include "machine.h"
#include "robot.h"

bool finishedExploration(Machine tabMachine[]){
	bool tmp = true;
	for(int i=0; i<NBR_MACHINES; i++)
	{
		if(!tabMachine[i].getProcessed())
		{
			tmp = false;
		}
	}
	return tmp;
}

void updateZone(Machine (&tabMachine)[NBR_MACHINES], Robot tabRobot[]){
	for(int i=0; i<NBR_MACHINES; i++)
	{
		tabMachine[i].updateMachine(tabRobot);
	}
}

