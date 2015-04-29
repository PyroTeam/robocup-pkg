#include "ros/ros.h"
#include "manager_msg/order.h"
#include "manager_msg/activity.h"
#include "std_msgs/String.h"
#include <cstdlib>


void chatterCallback(const manager_msg::activity & msg)
{
	ROS_INFO("I heard: [%d]", msg.nb_robot);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "envoi_ordre");
	if (argc != 6)
	{
	ROS_INFO("usage: envoi_ordre numero_robot numero_ordre type parametre id");
	return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<manager_msg::order>("order");
	manager_msg::order srv;
	  
	srv.request.number_robot = atoll(argv[1]);
	srv.request.number_order = atoll(argv[2]);
	srv.request.type = atoll(argv[3]);
	srv.request.parameter = atoll(argv[4]);
	srv.request.id = atoll(argv[5]);
 
	srv.response.id = atoll(argv[5]);
	srv.response.id = 1;

	if (client.call(srv))
	{
		ROS_INFO("Etat: %d, Id: %d", (int)srv.response.accepted, (int)srv.response.id);
	}
	else
	{
		ROS_ERROR("Failed to call service ordre");
		return -1;
	}

	ros::Subscriber sub = n.subscribe("task_exec_state",1000,chatterCallback);
	ros::spin();  

	return 0;
}
