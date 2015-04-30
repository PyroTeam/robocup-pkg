#ifndef BUMPERLISTENER_H
#define BUMPERLISTENER_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"

class BumperListener {

public:

    BumperListener();
    int get_state(){return m_state;}
    void blCallback(const std_msgs::Bool &msg);


private:
    
    ros::NodeHandle m_nh;
    ros::Subscriber m_bl_sub;
    bool m_state;
    
};

#endif
