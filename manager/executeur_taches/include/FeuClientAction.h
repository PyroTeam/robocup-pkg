#ifndef FEUCLIENTACTION_H
#define FEUCLIENTACTION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <manager_msg/processLightSignalAction.h>
#include <manager_msg/LightSpec.h>


class FeuClientAction{
public :
	FeuClientAction();
	virtual  ~FeuClientAction();
	void lightsStates(std::vector<manager_msg::LightSpec> &m_lightSpec);
//private :
	//manager_msg::LightSpec m_lightSpec[];
};
#endif