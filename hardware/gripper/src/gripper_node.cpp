#include <ros.h>
#include <gripper_msg/SetGripper.h>
#include <gripper_msg/Grip.h>
#include <gripper_msg/GripperStatus.h>

gripper_msg::GripperStatus g_gripStatus;

enum TakingProcess
{
  IDLE,
  TURN,
  PUSH,
  ERROR
};

enum ReleasingProcess
{
  IDLE,
  BACKWARD,
  TURN,
  FORWARD,
  RELEASE
};

TakingProcess g_tp = TakingProcess::IDLE;
ReleasingProcess g_rp = ReleasingProcess::IDLE;

void gripperCallback(const gripper_msg::GripperStatus &status)
{
  g_gripStatus = status;

  if (g_gripStatus.error)
  {
    g_tp = TakingProcess::ERROR;
  }
}

bool gripper(gripper_msg::Grip::Request &req)
{
  gripper_msg::SetGripper grip;
  approche_finale_msg::Move mv;

  // Process to take something
  if (req.cmd = gripper_msg::Grip::TAKE)
  {
    switch (g_tp)
    {
      case IDLE:
        g_tp = TakingProcess::TURN;
      break;

      case TURN:
        if (g_gripStatus.statusPercentTurn < 100)
        {
          // turn the high servo to get the piece
          grip.turn = true;
          grip.push = false;
          grip.open = false;
        }
        else
        {
          g_tp = TakingProcess::PUSH;
        }
      break;

      case PUSH:
        if (g_gripStatus.statusPercentPush < 100)
        {
          // push the low servo to hold the piece
          grip.turn = true;
          grip.push = true;
          grip.open = false;
        }
        else
        {
          g_tp = TakingProcess::IDLE;
        }
      break;

      case ERROR:
        ROS_ERROR("Error in grip process");
      break;
    }
  }

  // Process to let something
  if (req.cmd = gripper_msg::Grip::LET)
  {
    switch (g_rp)
    {
      case IDLE:
        g_rp = ReleasingProcess::BACKWARD;
      break;

      case BACKWARD:
        if (FA_client.feedback < 100)
        {
          mv.cmd = -2.0;
        }
        else
        {
          g_rp = ReleasingProcess::RELEASE;
        }
      break;

      case RELEASE:
        if (g_gripStatus.statusPercentPush < 100)
        {
          // turn the low servo to release the piece
          grip.turn = true;
          grip.push = false;
          grip.open = true;
        }
        else
        {
          g_rp = ReleasingProcess::FORWARD;
        }
      break;

      case FORWARD:
        if (g_gripStatus.statusPercentTurn < 100)
        {
          mv.cmd = 2.0;
        }
        else
        {
          g_rp = ReleasingProcess::TURN;
        }
      break;

      case TURN:
        if (g_gripStatus.statusPercentTurn < 100)
        {
          grip.turn = true;
          grip.push = false;
          grip.open = true;
        }
        else
        {
          g_rp = ReleasingProcess::IDLE;
        }
      break;

      case ERROR:
        ROS_ERROR("Error in grip process");
      break;
    }
  }

  if (grip_client.call(grip))
  {
    ROS_INFO("Gripper connected");
  }
  else
  {
    ROS_ERROR("Failed to call service of low level gripper");
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
  ros::ServiceClient grip_client = n.serviceClient<gripper_msg::SetGripper>("hardware/low_level/gripper_srv");
  ros::ServiceClient FA_client = n.serviceClient<approche_finale_msg::Move>("hardware/gripper_srv");

  ros::ServiceServer gripper_server 	= n.advertiseService("hardware/high_level/gripper_srv", gripper);


  ros::Rate loop_rate(30);

  // Spin
  ros::spin();
  loop_rate.sleep();

  return 0;
}
