/**
 * \file 		orderListener_node.cpp
 * \brief		Programme de test du serveur pour le service manager_msg::order
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#include "ros/ros.h"
#include "manager_msg/order.h"

bool add(manager_msg::order::Request  &req,manager_msg::order::Response &res)
{
	res.id = req.id;
	res.number_order = req.number_order;
	res.number_robot = req.number_robot;
	res.accepted = 1;
	res.needToResendOrder = 0;
	ROS_INFO("request: Numero Robot=%d, Numero_ordre=%d, Type=%d, Parametre=%d, Id=%d",(int)req.number_robot,(int)req.number_order,(int)req.type,(int)req.parameter,(int)req.id);
	ROS_INFO("Etat:%d, Numero_robot=%d, Numero_ordre=%d Id=%d",(int)res.accepted,(int)res.number_robot,(int)res.number_order,(int)res.id);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "orderListener");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("manager/order", add);
	ROS_INFO("Ready to accept an order");
	ros::spin();

	return 0;
}
