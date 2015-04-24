#include <ros/ros.h>
#include <manager_msg/finalApproachingAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<manager_msg::finalApproachingAction> Server;

void execute(const manager_msg::finalApproachingGoalConstPtr &goal, Server* as){
	manager_msg::finalApproachingResult m_result;
	m_result.result = true;
	as->setSucceeded(m_result);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"final_approaching_server");
	ros::NodeHandle n;
	Server server(n,"finalApproaching",boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}