#include "FeuClientAction.h"

FeuClientAction::FeuClientAction() { 
}
FeuClientAction::~FeuClientAction(){}

void FeuClientAction::feu(){
	
	actionlib::SimpleActionClient<manager_msg::processLightSignalAction> client("processLightSignal",true);

	ROS_INFO("Waiting for action Server to start");

	client.waitForServer(); 

	ROS_INFO("Action server started, sending goal");

	manager_msg::processLightSignalGoal goal;
	client.sendGoal(goal);  // no goal for this action

	//wait for the action to return
	bool finished_before_timeout = client.waitForResult(ros::Duration(5.0));

	if(finished_before_timeout){
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished : %s ",state.toString().c_str());

	}
	else{
		ROS_INFO("Action didn't finish before the time out");
	}
	/*if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		printf("No problem and the current state is \n", client.getState().toString().c_str());
	}*/

}