#if (ARDUINO >= 100)
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <gripper_msg/SetGripper.h>
#include <gripper_msg/GripperStatus.h>

ros::NodeHandle nh;

Servo servoTurn;
Servo servoPush;

enum GripperCommand
{
	OPEN,
	CLOSE,
	FORCE_OPEN,
	ENDMVT,
	UNFORCE,
	ERROR,
	NONE
};

enum GripperState
{
	OPENED,
	CLOSED,
	OPENING,
	CLOSING,
	FAULTY
};

GripperCommand grip_cmd_turn, grip_cmd_push;
GripperState grip_state_turn, grip_state_push;

#define OPEN_VALUE_TURN	 150
#define CLOSE_VALUE_TURN  65
#define OPEN_VALUE_PUSH	 180
#define CLOSE_VALUE_PUSH  0

int targetTurn  = OPEN_VALUE_TURN;
int currentTurn = OPEN_VALUE_TURN;
int targetPush  = OPEN_VALUE_PUSH;
int currentPush = OPEN_VALUE_PUSH;

int diffValueTurn = (OPEN_VALUE_TURN - CLOSE_VALUE_TURN);
int diffValuePush = (OPEN_VALUE_PUSH - CLOSE_VALUE_PUSH);
int b_turn = - CLOSE_VALUE_TURN * 100 / diffValueTurn;
int b_push = - CLOSE_VALUE_PUSH * 100 / diffValuePush;
int percentTurn = 0;
int percentPush = 0;
boolean force = false;

void callback(gripper_msg::SetGripperRequest & req,
		          gripper_msg::SetGripperResponse & res)
{
	if (req.turn && req.open)
	{
		grip_cmd_turn = OPEN;
	}
	else if (req.turn && !req.open)
	{
		grip_cmd_turn = CLOSE;
	}

	if (req.push && req.open)
	{
		grip_cmd_push = OPEN;
	}
	else if (req.push && !req.open)
	{
		grip_cmd_push = CLOSE;
	}
}

ros::ServiceServer<gripper_msg::SetGripperRequest, gripper_msg::SetGripperResponse> gripper_server("hardware/low_level/gripper_srv", &callback);
gripper_msg::GripperStatus gripperStatus_msg;
ros::Publisher gripper_pub("hardware/gripper_status", &gripperStatus_msg);

void setup()
{
	servoTurn.attach(9);  // attaches the turn servo on pin 9 to the servo object
	servoTurn.write(targetTurn); //initialize the servo

	servoPush.attach(10);  // attaches the push servo on pin 10 to the servo object
	servoPush.write(targetPush); //initialize the servo

	grip_cmd_turn = NONE;
	grip_cmd_push = NONE;
	grip_state_turn = OPENED;
	grip_state_push = OPENED;

	nh.initNode();
	nh.advertiseService(gripper_server);
	nh.advertise(gripper_pub);
}

int subFreq = 5;
int iterFreq = 0;

void loop()
{
	//updatecmd
	switch (grip_cmd_turn)
	{
	case OPEN:
		if (grip_state_turn == CLOSED || grip_state_turn == CLOSING)
		{
			grip_state_turn = OPENING;
			targetTurn = OPEN_VALUE_TURN;
		}
		break;
	case CLOSE:
		if (force == false && (grip_state_turn == OPEN || grip_state_turn == OPENING))
		{
			grip_state_turn = CLOSING;
			targetTurn = CLOSE_VALUE_TURN;
		}
		break;
	case FORCE_OPEN:
		grip_state_turn = OPENING;
		targetTurn = OPEN_VALUE_TURN;
		force = true;
		break;
	case ENDMVT:
		if (grip_state_turn == OPENING)
		{
			grip_state_turn = OPENED;
		}
		else if (grip_state_turn == CLOSING)
		{
			grip_state_turn = CLOSED;
		}
		break;
	case UNFORCE:
		force = false;
		break;
	case ERROR:
		break;
	default:
	case NONE:
		break;
	}
	grip_cmd_turn = NONE;


	switch (grip_cmd_push)
	{
	case OPEN:
		if (grip_state_push == CLOSED || grip_state_push == CLOSING)
		{
			grip_state_push = OPENING;
			targetPush = OPEN_VALUE_PUSH;
		}
		break;
	case CLOSE:
		if (force == false && (grip_state_push == OPEN || grip_state_push == OPENING))
		{
			grip_state_push = CLOSING;
			targetPush = CLOSE_VALUE_PUSH;
		}
		break;
	case FORCE_OPEN:
		grip_state_push = OPENING;
		targetPush = OPEN_VALUE_PUSH;
		force = true;
		break;
	case ENDMVT:
		if (grip_state_push == OPENING)
		{
			grip_state_push = OPENED;
		}
		else if (grip_state_push == CLOSING)
		{
			grip_state_push = CLOSED;
		}
		break;
	case UNFORCE:
		force = false;
		break;
	case ERROR:
		break;
	default:
	case NONE:
		break;
	}
	grip_cmd_push = NONE;

	//update state
	switch (grip_state_turn)
	{
	case OPENING:
		if (currentTurn == targetTurn)
		{
			grip_cmd_turn = ENDMVT;
		}
		else if (currentTurn < targetTurn)
		{
			currentTurn = currentTurn + 1;
		}
		else // if (currentTurn > targetTurn)
		{
			currentTurn = targetTurn;
		}
		break;
	case CLOSING:
		if (currentTurn == targetTurn)
		{
			grip_cmd_turn = ENDMVT;
		}
		else if (currentTurn < targetTurn)
		{
			currentTurn = targetTurn;
		}
		else // if (currentTurn > targetTurn)
		{
			currentTurn = currentTurn - 1;
		}
		break;
	case FAULTY:
	case OPENED:
	case CLOSED:
	default:
		break;
	}


	switch (grip_state_push)
	{
	case OPENING:
		if (currentPush == targetPush)
		{
			grip_cmd_push = ENDMVT;
		}
		else if (currentPush < targetPush)
		{
			currentPush = currentPush + 1;
		}
		else // if (currentPush > targetPush)
		{
			currentPush = targetPush;
		}
		break;
	case CLOSING:
		if (currentPush == targetPush)
		{
			grip_cmd_push = ENDMVT;
		}
		else if (currentPush < targetPush)
		{
			currentPush = targetPush;
		}
		else // if (currentPush > targetPush)
		{
			currentPush = currentPush - 1;
		}
		break;
	case FAULTY:
	case OPENED:
	case CLOSED:
	default:
		break;
	}

	servoTurn.write(currentTurn);
	servoPush.write(currentPush);

	percentTurn = currentTurn * 100 / diffValueTurn + b_turn;
	percentPush = currentPush * 100 / diffValuePush + b_push;

	if (!iterFreq)
	{
		gripperStatus_msg.timeStamp = nh.now();
    gripperStatus_msg.moving = false;
		switch (grip_state_turn)
		{
  		case OPENED:
      case CLOSED:
      break;

  		case CLOSING:
  		case OPENING:
  			gripperStatus_msg.moving = true;
  		break;

      case FAULTY:
        gripperStatus_msg.error = true;
      break;
		}

		switch (grip_state_push)
    {
      case OPENED:
      case CLOSED:
      break;

      case CLOSING:
      case OPENING:
        gripperStatus_msg.moving = true;
      break;
      case FAULTY:
        gripperStatus_msg.error = true;
      break;
    }

		gripperStatus_msg.statusPercentTurn = percentTurn;
		gripperStatus_msg.statusPercentPush = percentPush;
		gripperStatus_msg.forced = force;
		gripper_pub.publish(&gripperStatus_msg);
	}
	iterFreq = (++iterFreq) % subFreq;

	nh.spinOnce();
	delay(20);

}
