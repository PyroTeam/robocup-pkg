#include "ros/ros.h"
#include <manager_msg/processLightSignalAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<manager_msg::processLightSignalAction> Server;

void execute(const manager_msg::processLightSignalGoalConstPtr &goal, Server* as){
	as->setSucceeded();
}

int main(int argc, char** argv){
	ros::init(argc,argv,"process_light_signal_server");
	ros::NodeHandle n;
	Server server(n,"processLightSignal",boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}