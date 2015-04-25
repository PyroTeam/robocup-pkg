#ifndef SRVORDER_H
#define SRVORDER_H

#include <ros/ros.h>

#include "manager_msg/order.h"

class Srvorder {

public:
    
    Srvorder(ros::Time game_time,int nb_order,int nb_robot,int type,int parametre,int id);
    ~Srvorder();
    
    bool get_accepted(){return m_accepted;}
    int get_number_order(){return m_number_order;}
    int get_number_robot(){return m_number_robot;}
    int get_id(){return m_id;}

private:
    ros::NodeHandle m_nh;
    ros::ServiceClient m_client;
    manager_msg::order m_srv;
    
    bool m_accepted;
    int m_number_order;
    int m_number_robot;
    int m_id;
  
};

#endif 
