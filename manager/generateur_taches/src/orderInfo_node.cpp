#include "orderInfo.h"
#include <ros/ros.h>
#include "comm_msg/Order.h"
#include "comm_msg/OrderInfo.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "orderInfo_node");
	OrderInfo orderInfo;
	ros::Rate loop_rate(10);
    
	while (ros::ok())
    {
		ros::spinOnce();
		loop_rate.sleep();
    }
	return 0;
}
