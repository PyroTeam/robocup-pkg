/**
 * \file 			orderInfo.h
 * \class			OrderInfo
 * \brief			classe récupérant les ordres de la RefBox
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-05-10
 * \copyright       2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef ORDERINFO_H
#define ORDERINFO_H

#include <ros/ros.h>
#include <vector>
#include "comm_msg/Order.h"
#include "comm_msg/OrderInfo.h"


class OrderInfo{

public:

	OrderInfo();
	
	std::vector<comm_msg::Order> getOrders(){return m_orders;}
	
	void orderInfoCallback(const comm_msg::OrderInfo::ConstPtr &msg);
	
	  
private:

	ros::NodeHandle m_nh;
    ros::Subscriber m_orderInfoSub;

    std::vector<comm_msg::Order> m_orders;

};

#endif
