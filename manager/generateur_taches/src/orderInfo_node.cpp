#include "orderInfo.h"
#include <ros/ros.h>
#include <vector>
#include "comm_msg/Order.h"
#include "comm_msg/OrderInfo.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "orderInfo_node");
	OrderInfo orderInfo;
	ros::Rate loop_rate(10);
    std::vector<comm_msg::Order> tabOrders = orderInfo.getOrders();
    int id = -1;
	while (ros::ok())
    {
    	tabOrders = orderInfo.getOrders();
    	//ROS_INFO("taille de tabOrders : %d",(int)tabOrders.size());
   		ROS_INFO_COND(!tabOrders.empty(),"id: %d",tabOrders[0].id);
		ros::spinOnce();
		loop_rate.sleep();
    }
	return 0;
}
