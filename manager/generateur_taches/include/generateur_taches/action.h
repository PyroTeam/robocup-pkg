#ifndef ACTION_H
#define ACTION_H

#include <ros/ros.h>
#include "robot.h"
#include "manager_msg/activity.h"

class Action {

public:

	Action();
    
    int getNbRobot(){return m_nbRobot;}
    int getState(){return m_state;}
    int getUsedMachine(){return m_usedMachine;}
    int getNbOrder(){return m_nbOrder;}
    
    void tesCallback(const manager_msg::activity &msg);
    void updateRobot(Robot (&robot)[3]);


private:
    
    ros::NodeHandle m_nh;
    ros::Subscriber m_activitySub;
    
    int m_nbRobot;
    int m_state;
    int m_usedMachine;
    int m_nbOrder;
    
};

#endif
