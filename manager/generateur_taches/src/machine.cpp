#include "machine.h"
#include "robot.h"

Machine::Machine(){
        m_x = 0;
        m_y = 0;
        m_processed = false;
        m_robot = 0;
}

void Machine::correspondanceZone(){
        if((m_x > -12) && (m_x <= -6))
                m_robot = 1;
        if((m_x > -6) && (m_x <= 6))
                m_robot = 2;
        if((m_x > 6) && (m_x < 12))
                m_robot = 3;
}

void Machine::updateMachine(Robot tabRobot[]){
        if(tabRobot[0].getOccuped() && tabRobot[1].getOccuped() && tabRobot[2].getOccuped())
                m_robot = -10;
        if(!tabRobot[0].getOccuped() && tabRobot[1].getOccuped() && tabRobot[2].getOccuped())
                m_robot = 0;
        if(tabRobot[0].getOccuped() && !tabRobot[1].getOccuped() && tabRobot[2].getOccuped())
                m_robot = 1;
        if(!tabRobot[0].getOccuped() && !tabRobot[1].getOccuped() && tabRobot[2].getOccuped())
                m_robot = 0;
        if(tabRobot[0].getOccuped() && tabRobot[1].getOccuped() && !tabRobot[2].getOccuped())
                m_robot = 2;
        if(!tabRobot[0].getOccuped() && tabRobot[1].getOccuped() && !tabRobot[2].getOccuped())
                m_robot = 2; 
        if(tabRobot[0].getOccuped() && !tabRobot[1].getOccuped() && !tabRobot[2].getOccuped())
                m_robot = 1;
        if(!tabRobot[0].getOccuped() && !tabRobot[1].getOccuped() && !tabRobot[2].getOccuped())
                m_robot = 0;       
}

