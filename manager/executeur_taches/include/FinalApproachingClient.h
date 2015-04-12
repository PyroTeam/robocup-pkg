#ifndef FINALAPPROACHINGCLIENT_H
#define FINALAPPROACHINGCLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <cstdint>

#include <manager_msg/finalApproachingAction.h>


class FinalApproachingClient{
public :
	FinalApproachingClient();
	virtual  ~FinalApproachingClient();
	void starting(int8_t machineType, int32_t machineSide);
};
#endif