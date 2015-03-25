#include "machine.h"
#include "robot.h"

Machine::Machine(){
        m_x = 0;
        m_y = 0;
        m_traite = false;
        m_robot = 0;
}

void Machine::correspondance_zone(){
        if((m_x > -12) && (m_x <= -6))
                m_robot = 1;
        if((m_x > -6) && (m_x <= 6))
                m_robot = 2;
        if((m_x > 6) && (m_x < 12))
                m_robot = 3;
}

void Machine::update_machine(Robot tab_robot[]){
        if(tab_robot[0].get_occupe() && tab_robot[1].get_occupe() && tab_robot[2].get_occupe())
                m_robot = -10;
        if(!tab_robot[0].get_occupe() && tab_robot[1].get_occupe() && tab_robot[2].get_occupe())
                m_robot = 0;
        if(tab_robot[0].get_occupe() && !tab_robot[1].get_occupe() && tab_robot[2].get_occupe())
                m_robot = 1;
        if(!tab_robot[0].get_occupe() && !tab_robot[1].get_occupe() && tab_robot[2].get_occupe())
                m_robot = 0;
        if(tab_robot[0].get_occupe() && tab_robot[1].get_occupe() && !tab_robot[2].get_occupe())
                m_robot = 2;
        if(!tab_robot[0].get_occupe() && tab_robot[1].get_occupe() && !tab_robot[2].get_occupe())
                m_robot = 2; 
        if(tab_robot[0].get_occupe() && !tab_robot[1].get_occupe() && !tab_robot[2].get_occupe())
                m_robot = 1;
        if(!tab_robot[0].get_occupe() && !tab_robot[1].get_occupe() && !tab_robot[2].get_occupe())
                m_robot = 0;       
}

