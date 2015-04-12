#include "FinalApproachingClient.h"

FinalApproachingClient::FinalApproachingClient() { 
}
FinalApproachingClient::~FinalApproachingClient(){}

void FinalApproachingClient::starting(int8_t machineType, int32_t machineSide){
	
	actionlib::SimpleActionClient<manager_msg::finalApproachingAction> client("finalApproaching",true);

	ROS_INFO("Waiting for action Server to start");

	client.waitForServer(); 

	ROS_INFO("Action server started, sending goal");

	manager_msg::finalApproachingGoal goal;
	goal.type = machineType;
	goal.side = machineSide;
	client.sendGoal(goal);  

	//wait for the action to return
	bool finished_before_timeout = client.waitForResult(ros::Duration(5.0));

	if(finished_before_timeout){
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished : %s ",state.toString().c_str());

	}
	else{
		ROS_INFO("Action didn't finish before the time out");
	}

}




