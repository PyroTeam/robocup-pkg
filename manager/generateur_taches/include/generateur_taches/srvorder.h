/**
 * \file 		srvorder.h
 * \class		Srvorder
 * \brief		classe permettant le traitement du service manager_msg::order
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef SRVORDER_H
#define SRVORDER_H

#include <ros/ros.h>

#include "manager_msg/order.h"

class Srvorder {

public:

    Srvorder(ros::Time game_time,int nb_order,int nb_robot,int type,int parameter,int id);
    ~Srvorder();

    bool getAccepted(){return m_accepted;}
    bool getNeedToResendOrder(){return m_needToResendOrder;}
    int getNumberOrder(){return m_numberOrder;}
    int getNumberRobot(){return m_numberRobot;}
    int getId(){return m_id;}

private:

    ros::NodeHandle m_nh;
    ros::ServiceClient m_client;
    manager_msg::order m_srv;

    bool m_accepted;
    bool m_needToResendOrder;
    int m_numberOrder;
    int m_numberRobot;
    int m_id;

};

#endif
