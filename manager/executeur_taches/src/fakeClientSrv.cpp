#include "fakeClientSrv.h"

  fakeClientSrv::fakeClientSrv() {}
  fakeClientSrv::~fakeClientSrv(){}

bool fakeClientSrv::gripper_uppdate(){

  ros::init(argc, argv, "gripper_uppdate");
  if (argc != 2)
  {
    ROS_INFO("usage: gripper_uppdate");
    return true;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<manager_msg::SetGripper>("/fakeRobotino/setGripper");
  manager_msg::SetGripper srv;
  srv.request.state = atoll(argv[1]);
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

