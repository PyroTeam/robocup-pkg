#include "GripperClientSrv.h"

GripperClientSrv::GripperClientSrv()
{

}

GripperClientSrv::~GripperClientSrv()
{

}

bool GripperClientSrv::gripper_update(bool new_state)
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gripper_msg::SetGripper>("fakeRobotino/setGripper");
	gripper_msg::SetGripper srv;
	srv.request.state = new_state;
	if (client.call(srv))
	{
		ROS_INFO("State: No problem");
	}
	else
	{
		ROS_ERROR("Failed to call service gripper_uppdate");
		return true;
	}
	return false;
}
