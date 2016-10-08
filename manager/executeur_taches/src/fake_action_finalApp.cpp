#include <ros/ros.h>
#include <final_approach_msg/FinalApproachingAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<final_approach_msg::FinalApproachingAction> Server;

void execute(const final_approach_msg::FinalApproachingGoalConstPtr &goal, Server* as)
{
	final_approach_msg::FinalApproachingResult m_result;
	m_result.result = true;
	as->setSucceeded(m_result);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"final_approaching_server");
	ros::NodeHandle n;
	Server server(n,"navigation/finalApproaching",boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}