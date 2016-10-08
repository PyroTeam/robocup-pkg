#include "GripperClientSrv.h"

GripperClientSrv::GripperClientSrv()
{

}

GripperClientSrv::~GripperClientSrv()
{

}

bool GripperClientSrv::grip()
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gripper_msg::Grip>("fakeRobotino/Grip");
    gripper_msg::Grip srv;

    srv.request.cmd = gripper_msg::GripRequest::TAKE;

    if (client.call(srv))
    {
        ROS_INFO("Taking: No problem");
    }
    else
    {
        ROS_ERROR("Failed to call service TAKE");
        return true;
    }
    return false;
}

bool GripperClientSrv::let()
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gripper_msg::Grip>("fakeRobotino/Grip");
    gripper_msg::Grip srv;

    srv.request.cmd = gripper_msg::GripRequest::LET;

    if (client.call(srv))
    {
        ROS_INFO("Letting: No problem");
    }
    else
    {
        ROS_ERROR("Failed to call service LET");
        return true;
    }
    return false;
}
