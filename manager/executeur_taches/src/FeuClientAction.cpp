#include "FeuClientAction.h"

FeuClientAction::FeuClientAction() {
}
FeuClientAction::~FeuClientAction(){}

void FeuClientAction::lightsStates(std::vector<comm_msg::LightSpec> &m_lightSpec){

	int i=0;

	actionlib::SimpleActionClient<trait_im_msg::processLightSignalAction> client("lecture_feu",true);

	ROS_INFO("Waiting for action Lights Server to start");

	client.waitForServer();

	ROS_INFO("Action server started, sending goal");

	trait_im_msg::processLightSignalGoal goal;
	client.sendGoal(goal);  // no goal for this action

	//wait for the action to return
	bool finished_before_timeout = client.waitForResult(ros::Duration(5.0));

	if(finished_before_timeout){
		actionlib::SimpleClientGoalState state = client.getState();
		ROS_INFO("Action finished : %s ",state.toString().c_str());
		m_lightSpec = client.getResult()->light_signal;
		/*for(i = 0 ; i < m_lightSpec.size() ; i++){
		 	ROS_INFO("color : %d , state : %d",m_lightSpec[i].color,m_lightSpec[i].state);
		}*/

	}
	else{
		ROS_INFO("Action didn't finish before the time out");
	}

}
