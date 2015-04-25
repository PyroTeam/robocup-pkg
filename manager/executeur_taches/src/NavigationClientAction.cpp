#include "NavigationClientAction.h"

NavigationClientAction::NavigationClientAction() {
}
NavigationClientAction::~NavigationClientAction(){}

int NavigationClientAction::goToAPoint(geometry_msgs::Pose2D dest_point){

	actionlib::SimpleActionClient<deplacement_msg::MoveToPoseAction> client("moveToPose",true);

	ROS_INFO("Waiting for action Navigation Server to start");

	client.waitForServer();

	ROS_INFO("Action server started, sending goal");

	deplacement_msg::MoveToPoseGoal goal;
	goal.position_finale = dest_point;
	client.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = client.waitForResult(ros::Duration(5.0));

	if(finished_before_timeout){
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished : %s ",state.toString().c_str());
	}
	else{
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished but FAILED: %s ",state.toString().c_str());
	}
	
	return client.getResult()->result;

}
