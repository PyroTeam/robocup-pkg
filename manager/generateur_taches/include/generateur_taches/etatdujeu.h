#ifndef ACTION_H
#define ACTION_H

#include <ros/ros.h>
#include "robot.h"
#include "manager_msg/activity.h"

class Action {

public:

    Action();
    
    int get_nb_robot(){return m_nb_robot;}
    int get_state(){return m_state;}
    int get_machine_used(){return m_machine_used;}
    int get_nb_order(){return m_nb_order;}
    
    void tesCallback(const manager_msg::activity &msg);
    void update_robot(Robot (&robot)[3]);


private:
    
    ros::NodeHandle m_nh;
    ros::Subscriber m_activity_sub;
    
    int m_nb_robot;
    int m_state;
    int m_machine_used;
    int m_nb_order;
    
};

#endif
