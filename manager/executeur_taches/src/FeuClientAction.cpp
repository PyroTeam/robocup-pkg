#include "FeuClientAction.h"

FeuClientAction::FeuClientAction() {}

FeuClientAction::~FeuClientAction(){}

void FeuClientAction::lightsStates(std::vector<comm_msg::LightSpec> &lightSpec)
{
    int i=0;

    actionlib::SimpleActionClient<trait_im_msg::processLightSignalAction> client("computerVision/lecture_feu",true);

    ROS_INFO("Waiting for action Lights Server to start");

    client.waitForServer();

    ROS_INFO("Action server started, sending goal");

    trait_im_msg::processLightSignalGoal goal;
    client.sendGoal(goal);  // empty goal for this action

    //wait for the action to return
    bool finished_before_timeout = client.waitForResult(ros::Duration(5.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished : %s ",state.toString().c_str());
        lightSpec = client.getResult()->light_signal;
    }
    else
    {
        ROS_INFO("Action didn't finish before the time out");
        client.cancelGoal();
    }
}
