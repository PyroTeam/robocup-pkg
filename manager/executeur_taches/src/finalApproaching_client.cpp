#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <manager_msg/finalApproachingAction.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_finalApproaching");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<manager_msg::finalApproachingAction> ac("navigation/finalApproaching", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	manager_msg::finalApproachingGoal goal;
	goal.type = 3;
	goal.side = 100;
	goal.parameter = 40;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}
	return 0;
}
