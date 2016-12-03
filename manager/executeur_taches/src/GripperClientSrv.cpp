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
    ros::ServiceClient client = n.serviceClient<robotino_msgs::Grip>("fakeRobotino/Grip");
    robotino_msgs::Grip srv;

    srv.request.cmd = robotino_msgs::GripRequest::TAKE;

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
    ros::ServiceClient client = n.serviceClient<robotino_msgs::Grip>("fakeRobotino/Grip");
    robotino_msgs::Grip srv;

    srv.request.cmd = robotino_msgs::GripRequest::LET;

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
