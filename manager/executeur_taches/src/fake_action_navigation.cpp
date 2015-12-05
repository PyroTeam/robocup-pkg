#include <ros/ros.h>
#include <deplacement_msg/MoveToPoseAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<deplacement_msg::MoveToPoseAction> Server;

void execute(const deplacement_msg::MoveToPoseGoalConstPtr &goal, Server* as)
{
	deplacement_msg::MoveToPoseResult m_result;
	m_result.result = deplacement_msg::MoveToPoseResult::FINISHED;
	as->setSucceeded(m_result);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"move_to_pose_server");
	ros::NodeHandle n;
	Server server(n,"navigation/moveToPose",boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}
