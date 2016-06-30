#include "FinalApproachingClient.h"

FinalApproachingClient::FinalApproachingClient() {}

FinalApproachingClient::~FinalApproachingClient(){}

void FinalApproachingClient::starting(int8_t machineType, int8_t machineSide, int8_t machineParameter)
{
	actionlib::SimpleActionClient<final_approach_msg::FinalApproachingAction> client("navigation/FinalApproach",true);

	ROS_INFO("Waiting for fa action Server to start");

	client.waitForServer();

	ROS_INFO("Action server started, sending goal");

	final_approach_msg::FinalApproachingGoal goal;
	goal.type = machineType;
	goal.side = machineSide;
	goal.parameter = machineParameter;
	client.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

	if(finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished : %s ",state.toString().c_str());

	}
	else
	{
		ROS_INFO("Action didn't finish before the time out");
	}
}
