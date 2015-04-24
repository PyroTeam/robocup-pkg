#include <ros/ros.h>
#include <manager_msg/MoveToPoseAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<manager_msg::MoveToPoseAction> Server;

void execute(const manager_msg::MoveToPoseGoalConstPtr &goal, Server* as){
	manager_msg::MoveToPoseResult m_result;
	m_result.result = manager_msg::MoveToPoseResult::FINISHED;
	as->setSucceeded(m_result);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"move_to_pose_server");
	ros::NodeHandle n;
	Server server(n,"MoveToPose",boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}