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
  START,
  TURN,
  PUSH,
  ERROR
};

enum class ReleasingProcess
{
  IDLE,
  START,
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

  //ROS_INFO("status received P [%d] | T [%d]", status.statusPercentPush, status.statusPercentTurn);

  if (g_gripStatus.error)
  {
    g_tp = TakingProcess::ERROR;
    g_rp = ReleasingProcess::ERROR;
  }
}

bool gripper(gripper_msg::GripRequest &req,
             gripper_msg::GripResponse &res)
{
  ROS_INFO("gripper callback");
  ROS_INFO("Request #%d received", req.cmd);
  
  // Process to take something
  if (req.cmd == gripper_msg::GripRequest::TAKE)
  {
    g_tp = TakingProcess::START;
  }

  // Process to let something
  if (req.cmd == gripper_msg::GripRequest::LET)
  {
    g_rp = ReleasingProcess::START;
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

  gripper_msg::SetGripper grip;
  FA_Client move;
  int feedback = 0;
  bool initPhase = true;

  bool oldServo, oldOpen;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    if (initPhase)
    {
      // si le servo haut n'est pas en position haute
      if(g_gripStatus.statusPercentTurn != 100)
      {
        // on le monte
        grip.request.servo = false;
        grip.request.open = true;
        ROS_WARN("T -- First phase of INIT");
      }

      // si le servo haut est bien en haut et que le servo bas n'est pas rentré
      if(g_gripStatus.statusPercentTurn == 100 && g_gripStatus.statusPercentPush != 0)
      {
        // on le rentre
        grip.request.servo = true;
        grip.request.open = false;
        ROS_WARN("T -- Second phase of INIT");
      }
      // Si les deux sont en position d'initialisation
      if (g_gripStatus.statusPercentTurn == 100 && g_gripStatus.statusPercentPush == 0)
      {
        initPhase = false;
      }
    }
    oldServo = grip.request.servo;
    oldOpen  = grip.request.open;

    ROS_INFO_ONCE("Init phase finished");

    switch (g_tp)
    {
      case TakingProcess::IDLE:
      break;

      case TakingProcess::START:
        ROS_INFO("Starting Taking Phase");
        g_tp = TakingProcess::TURN;
      break;

      case TakingProcess::TURN:
        ROS_INFO("Turning Phase");
        if (g_gripStatus.statusPercentTurn != 0)
        {
          // turn the high servo to get the piece
          grip.request.servo = false;
          grip.request.open = false;
          ROS_INFO("I am turning the servo to close");
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
          grip.request.servo = true;
          grip.request.open = true;
          ROS_INFO("I am pushing the piece");
        }
        else
        {
          ROS_INFO("Base taken");
          g_tp = TakingProcess::IDLE;
        }
      break;

      case TakingProcess::ERROR:
        ROS_ERROR("Error in grip process");
      break;
    }


    switch (g_rp)
    {
      case ReleasingProcess::IDLE:
      break;

      case ReleasingProcess::START:
        // g_rp = ReleasingProcess::BACKWARD;
        g_rp = ReleasingProcess::RELEASE;
      break;

      case ReleasingProcess::BACKWARD:
        /*feedback = move.starting(-2.0);

        if (feedback < 100)
        {
          ROS_ERROR("FAIL controlled final approaching");
        }
        else
        {
          ROS_INFO("Controlled final approaching SUCCESSFULL (backward)");
          g_rp = ReleasingProcess::RELEASE;
        }*/
      break;

      case ReleasingProcess::RELEASE:
        if (g_gripStatus.statusPercentPush != 0)
        {
          // turn the low servo to release the piece
          grip.request.servo = true;
          grip.request.open = false;
          ROS_INFO("I am releasing the piece");
        }
        else
        {
          // g_rp = ReleasingProcess::FORWARD;
          g_rp = ReleasingProcess::TURN;
        }
      break;

      case ReleasingProcess::FORWARD:
        /*feedback = move.starting(0.0);

        if (feedback < 100)
        {
          ROS_ERROR("FAIL controlled final approaching");
        }
        else
        {
          ROS_INFO("Controlled final approaching SUCCESSFULL (forward)");
          g_rp = ReleasingProcess::TURN;
        }*/
      break;

      case ReleasingProcess::TURN:
        if (g_gripStatus.statusPercentTurn != 100)
        {
          grip.request.servo = false;
          grip.request.open = true;
          ROS_INFO("I am turning the servo to open");
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

    if (g_grip_client.call(grip))
    {
      if (oldServo != grip.request.servo || oldOpen != grip.request.open)
      {
        ROS_INFO("Just send the setpoint for low level gripper (%d, %d)",grip.request.servo,grip.request.open);
      }
    }
    else
    {
      ROS_ERROR("Failed to call service of low level gripper");
    }

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
