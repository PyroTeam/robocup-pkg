#include "ros/ros.h"
#include <trait_im_msg/processLightSignalAction.h>
#include <comm_msg/LightSpec.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<trait_im_msg::processLightSignalAction> Server;

void execute(const trait_im_msg::processLightSignalGoalConstPtr &goal, Server* as){
	trait_im_msg::processLightSignalResult result;
	comm_msg::LightSpec light;
	// yellow
	light.color = light.YELLOW;
	light.state = light.ON;
	result.light_signal.push_back(light);
	// green
	light.color = light.GREEN;
	light.state = light.OFF;
	result.light_signal.push_back(light);
	// red
	light.color = light.RED;
	light.state = light.BLINK;
	result.light_signal.push_back(light);
	as->setSucceeded(result);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"process_light_signal_server");
	ros::NodeHandle n;
	Server server(n,"processLightSignal",boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}
