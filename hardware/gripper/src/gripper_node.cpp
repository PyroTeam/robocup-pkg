#include <ros.h>
#include <gripper_msg/SetGripper.h>
#include <gripper_msg/GripperStatus.h>

bool g_turned;
bool g_pushed;


enum TakingProcess
{
  TURN,
  PUSH,
  GO_BACK,
  UNKOWN
};

TakingProcess tp;

void gripperCallback(const gripper_msg::GripperStatus &status)
{
  g_turned = status.turn;
  g_pushed = status.push;
}

bool take(gripper_msg::Take::Request &req)
{
  if (req.take)
  {
    gripper_msg::SetGripper srv;
    // msg to be defined
    approche_finale_msg::Move mv;

    switch (tp)
    {
      case TURN:
        srv.stateTurn = true;
        srv.statePush = false;
      break;
      case PUSH:
        srv.stateTurn = true;
        srv.statePush = true;
      break;
      case GO_BACK:
        mv.distance = -1.0;
      case UNKNOWN:
        gripperStatus_msg.turn = false;
      break;
    }

    if (client.call(srv))
    {
      ROS_INFO("State: No problem");
    }
    else
    {
      ROS_ERROR("Failed to call service gripper turn");
      return true;
    }


    // send message to final approach to
  }

  return true;
}

int main(int argc, char** argv)
{
  ROS_INFO("Starting node gripper_node");

  // Initialisation du noeud ROS
  ros::init(argc, argv, "gripper_node");

  ros::NodeHandle n;

  // Souscription au topic /gripper_status
  ros::Subscriber sub_gripper  = n.subscribe("hardware/gripper_status", 1000, gripperCallback);

  // Connection en tant que client au noeud gripper
  ros::ServiceClient grip_client = n.serviceClient<gripper_msg::SetGripper>("hardware/gripper_srv");
  ros::ServiceClient FA_client = n.serviceClient<approche_finale_msg::Move>("hardware/gripper_srv");

  ros::ServiceServer take 	= n.advertiseService("take", take);
  ros::ServiceServer release 	= n.advertiseService("release", release);


  ros::Rate loop_rate(100);

  // Spin
  ros::spin();
  loop_rate.sleep();

  return 0;
}
