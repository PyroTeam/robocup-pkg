#include "tabmachine.h"
#include "machine.h"
#include "robot.h"

bool exploration_finie(Machine tab_machine[]){
        bool tmp = true;
        for(int i=0; i<NBR_MACHINES; i++){
                if(!tab_machine[i].get_traite())
                        tmp = false;
        }
        return tmp;
}

void update_zone(Machine (&tab_machine)[NBR_MACHINES], Robot tab_robot[]){
        for(int i=0; i<NBR_MACHINES; i++)
                tab_machine[i].update_machine(tab_robot);
}

