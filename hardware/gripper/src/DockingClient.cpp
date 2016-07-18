#include "DockingClient.h"

DockingClient::DockingClient()
{
}

DockingClient::~DockingClient()
{

}

int DockingClient::starting(double setpoint)
{
  // Instanciation de l'action Approche Finale
	actionlib::SimpleActionClient<final_approach_msg::FinalApproachingAction> client("navigation/finalApproaching_node",true);
	client.waitForServer();

	ROS_INFO("Wait for controlled approach");

  // I just ask a controlled approach
	final_approach_msg::FinalApproachingGoal goal;
	goal.mode = final_approach_msg::FinalApproachingGoal::CONTROLLED_BY_PARAM;
	goal.control_param = setpoint;
	client.sendGoal(goal);


	//wait for the action to return
  // duration to define wisely using control_param
	if(client.waitForResult(ros::Duration(10.0)))
	{
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished with state : %s ",state.toString().c_str());
    return 100;
	}
	else
	{
		ROS_INFO("Action didn't finish before the time out");
    return 0;
	}
}
