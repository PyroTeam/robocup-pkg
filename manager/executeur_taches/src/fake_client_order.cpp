#include <ros/ros.h>
#include "manager_msg/order.h"
#include <cstdlib>


int main(int argc, char **argv){
	ros::init(argc, argv, "order");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient <manager_msg::order>("order");
	manager_msg::order srv;
	ros::Time game_time = ros::Time::now();
	srv.request.number_order = 9;
	srv.request.number_robot = 0;
	srv.request.type = 0; // TAKE_BASE
	srv.request.parameter = 10;
	srv.request.id = 2;
	if(client.call(srv)){
		if(!(srv.response.accepted)) ROS_INFO(" the server didn't accepted the server's request");
		else ROS_INFO("the server has responded : nb_order = %d, nb_robot = %d, id = %d ",(int)srv.response.number_order, (int)srv.response.number_robot, (int)srv.response.id );
	}
	else{
		ROS_ERROR("Failed to call service order ");
		return 1;
	}
	return 0;
}

