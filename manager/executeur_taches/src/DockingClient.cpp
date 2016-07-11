#include "DockingClient.h"

DockingClient::DockingClient()
: m_nh()
, m_globalTimeout(m_nh, "navigation/Docking/globalTimeout", 15.0)
, m_lightAsservTimeout(m_nh, "navigation/Docking/lightAsservTimeout", 7.0)
, m_success(false)
{
}

DockingClient::~DockingClient(){}

void DockingClient::starting(int8_t machineType, int8_t machineSide, int8_t machineParameter)
{
	actionlib::SimpleActionClient<final_approach_msg::FinalApproachingAction> client("navigation/Docking",true);

	ROS_INFO("Waiting for fa action Server to start");

	client.waitForServer();

	ROS_INFO("Action server started, sending goal");

	final_approach_msg::FinalApproachingGoal goal;
	goal.type = machineType;
	goal.side = machineSide;
	goal.parameter = machineParameter;
	client.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = client.waitForResult(ros::Duration(m_globalTimeout()));

	if(finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = client.getState();
    m_success = client.getResult()->success;
		ROS_INFO("Action finished : %s ",state.toString().c_str());
	}
	else
	{
		ROS_WARN("Docking aborted due to timeout. Will continue");
        client.cancelGoal();
        m_success = true;
	}
}

bool DockingClient::getSuccess()
{
    return m_success;
}
