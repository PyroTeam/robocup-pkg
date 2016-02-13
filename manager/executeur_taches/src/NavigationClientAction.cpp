#include "NavigationClientAction.h"

NavigationClientAction::NavigationClientAction() 
{
}

NavigationClientAction::~NavigationClientAction(){}

deplacement_msg::finalApproaching NavigationClientAction::setFinalApproachingData(int8_t machineType, int8_t machineSide, int8_t machineParameter)
{
	deplacement_msg::finalApproaching fa_data;
	fa_data.type = machineType;
	fa_data.side = machineSide;
	fa_data.parameter = machineParameter;
	return fa_data;
}

bool NavigationClientAction::getSuccess()
{
	return m_success;
}

int16_t NavigationClientAction::getError()
{
	return m_error;
}

int16_t NavigationClientAction::getStatus()
{
	return m_status;
}

/* This function is used for the final approaching and for the displacement */
void NavigationClientAction::navigate(geometry_msgs::Pose2D dest_point, bool fa, bool fast, bool puckInGripper, bool goAway, int8_t machineType, int8_t machineSide, int8_t machineParameter)
{
	actionlib::SimpleActionClient<deplacement_msg::MoveToPoseAction> client("navigation/moveToPose",true);

	ROS_INFO("Waiting for action Navigation Server to start");

	client.waitForServer();

	ROS_INFO("Action server started, sending goal");

	deplacement_msg::MoveToPoseGoal goal;

	if(fa)
	{
		goal.faData = setFinalApproachingData(machineType, machineSide, machineParameter)
	}
	goal.position_finale = dest_point;
	goal.final_approaching = fa;
	goal.puck_in_gripper = puckInGripper;
	goal.fast = fast;
	goal.go_away = goAway;
	client.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = client.waitForResult(ros::Duration(5.0));

	if(finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished : %s ",state.toString().c_str());
	}
	else
	{
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished but FAILED: %s ",state.toString().c_str());
	}

	m_success = m_client.getResult()->sucess;
	m_error = m_client.getResult()->error;
	m_status = m_client.getResult()->status;
}
