#include "ros/ros.h"
#include "manager_msg/order.h"

bool add(manager_msg::order::Request &req,manager_msg::order::Response &res)
{
	res.id = req.id;
	res.number_order = req.number_order;
	res.number_robot = req.number_robot;
	res.accepted = 1;
	ROS_INFO("request: Numero Robot=%d, Numero_ordre=%d, Type=%d, Parametre=%d, Id=%d",(int)req.number_robot,(int)req.number_order,(int)req.type,(int)req.parameter,(int)req.id);
	ROS_INFO("Etat:%d, Numero_robot=%d, Numero_ordre=%d Id=%d",(int)res.accepted,(int)res.number_robot,(int)res.number_order,(int)res.id);
	return true;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "reception_ordre");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("order", add);
	ROS_INFO("Pret a recevoir des ordres.");
	ros::spin();
	return 0;
}
