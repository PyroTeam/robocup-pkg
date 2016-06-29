#include <ros/ros.h>
#include <FA_Client.h>
#include <gripper_msg/SetGripper.h>
#include <gripper_msg/Grip.h>
#include <gripper_msg/GripperStatus.h>

gripper_msg::GripperStatus g_gripStatus;
ros::ServiceClient g_grip_client;

enum class TakingProcess
{
  IDLE,
  TURN,
  PUSH,
  ERROR
};

enum class ReleasingProcess
{
  IDLE,
  BACKWARD,
  TURN,
  FORWARD,
  RELEASE,
  ERROR
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

bool gripper(gripper_msg::GripRequest &req,
             gripper_msg::GripResponse &res)
{
  gripper_msg::SetGripper grip;
  FA_Client move;
  int feedback;

  // Process to take something
  if (req.cmd = gripper_msg::GripRequest::TAKE)
  {
    switch (g_tp)
    {
      case TakingProcess::IDLE:
        g_tp = TakingProcess::TURN;
      break;

      case TakingProcess::TURN:
        if (g_gripStatus.statusPercentTurn < 100)
        {
          // turn the high servo to get the piece
          grip.request.turn = true;
          grip.request.push = false;
          grip.request.open = false;
          ROS_DEBUG("I am turning the servo to close");
        }
        else
        {
          g_tp = TakingProcess::PUSH;
        }
      break;

      case TakingProcess::PUSH:
        if (g_gripStatus.statusPercentPush < 100)
        {
          // push the low servo to hold the piece
          grip.request.turn = true;
          grip.request.push = true;
          grip.request.open = false;
          ROS_DEBUG("I am pushing the piece");
        }
        else
        {
          g_tp = TakingProcess::IDLE;
        }
      break;

      case TakingProcess::ERROR:
        ROS_ERROR("Error in grip process");
      break;
    }
  }

  // Process to let something
  if (req.cmd = gripper_msg::GripRequest::LET)
  {
    switch (g_rp)
    {
      case ReleasingProcess::IDLE:
        g_rp = ReleasingProcess::BACKWARD;
      break;

      case ReleasingProcess::BACKWARD:
        feedback = move.starting(-2.0);

        if (feedback < 100)
        {
          ROS_ERROR("FAIL controlled final approaching");
        }
        else
        {
          ROS_DEBUG("Controlled final approaching SUCCESSFULL (backward)");
          g_rp = ReleasingProcess::RELEASE;
        }
      break;

      case ReleasingProcess::RELEASE:
        if (g_gripStatus.statusPercentPush < 100)
        {
          // turn the low servo to release the piece
          grip.request.turn = true;
          grip.request.push = false;
          grip.request.open = true;
          ROS_DEBUG("I am releasing the piece");
        }
        else
        {
          g_rp = ReleasingProcess::FORWARD;
        }
      break;

      case ReleasingProcess::FORWARD:
        feedback = move.starting(0.0);

        if (feedback < 100)
        {
          ROS_ERROR("FAIL controlled final approaching");
        }
        else
        {
          ROS_DEBUG("Controlled final approaching SUCCESSFULL (forward)");
          g_rp = ReleasingProcess::TURN;
        }
      break;

      case ReleasingProcess::TURN:
        if (g_gripStatus.statusPercentTurn < 100)
        {
          grip.request.turn = true;
          grip.request.push = false;
          grip.request.open = true;
          ROS_DEBUG("I am turning the servo to open");
        }
        else
        {
          g_rp = ReleasingProcess::IDLE;
        }
      break;

      case ReleasingProcess::ERROR:
        ROS_ERROR("Error in grip process");
      break;
    }
  }

  if (g_grip_client.call(grip))
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
  g_grip_client = n.serviceClient<gripper_msg::SetGripper>("hardware/low_level/gripper_srv");

  ros::ServiceServer gripper_server = n.advertiseService("hardware/high_level/gripper_srv", gripper);


  ros::Rate loop_rate(30);

  // Spin
  ros::spin();
  loop_rate.sleep();

  return 0;
}
