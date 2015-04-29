#ifndef SRVORDER_H
#define SRVORDER_H

#include <ros/ros.h>

#include "manager_msg/order.h"

class Srvorder {

public:
    
    Srvorder(ros::Time game_time,int nb_order,int nb_robot,int type,int parameter,int id);
    ~Srvorder();
    
    bool getAccepted(){return m_accepted;}
    int getNumberOrder(){return m_numberOrder;}
    int getNumberRobot(){return m_numberRobot;}
    int getId(){return m_id;}

private:

    ros::NodeHandle m_nh;
    ros::ServiceClient m_client;
    manager_msg::order m_srv;
    
    bool m_accepted;
    int m_numberOrder;
    int m_numberRobot;
    int m_id;
  
};

#endif 
