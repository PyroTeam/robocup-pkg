#include "FinalApproachingClient.h"

FinalApproachingClient::FinalApproachingClient()
: m_nh()
, m_globalTimeout(m_nh, "navigation/finalApproach/globalTimeout", 15.0)
, m_lightAsservTimeout(m_nh, "navigation/finalApproach/lightAsservTimeout", 7.0)
, m_success(false)
{
}

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
  bool finished_before_timeout = client.waitForResult(ros::Duration(m_globalTimeout()));

  if(finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = client.getState();
    m_success = client.getResult()->success;
    ROS_INFO("Action finished : %s ",state.toString().c_str());

  }
  else
  {
    ROS_WARN("FinalApproach aborted due to timeout. Will continue");
    client.cancelGoal();
    m_success = true;
  }
}

bool FinalApproachingClient::getSuccess()
{
  return m_success;
}
