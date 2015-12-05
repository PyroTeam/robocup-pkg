#include "orderInfo.h"
#include <ros/ros.h>
#include "comm_msg/Order.h"
#include "comm_msg/OrderInfo.h"

void OrderInfo::orderInfoCallback(const comm_msg::OrderInfo::ConstPtr &msg)
{
	m_orders.clear();
	m_orders = msg->orders;
	//ROS_INFO("id: %d complexity: %d base_color: %d cap_color: %d",msg->orders[0].id,msg->orders[0].complexity,msg->orders[0].base_color,msg->orders[0].cap_color);
}

OrderInfo::OrderInfo()
{
	m_orderInfoSub = m_nh.subscribe("refBoxComm/OrderInfo",1000,&OrderInfo::orderInfoCallback,this);
}
