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
	FAULTY,
	UNKOWN
};

GripperCommand gc_turn, gc_push;
GripperState gs_turn, gs_push;

#define OPEN_VALUE_TURN	 150
#define CLOSE_VALUE_TURN  65
#define OPEN_VALUE_PUSH	 150
#define CLOSE_VALUE_PUSH  65

int targetTurn = OPEN_VALUE_TURN;
int currentTurn = OPEN_VALUE_TURN;
int targetPush = OPEN_VALUE_PUSH;
int currentPush = OPEN_VALUE_PUSH;

int diffValueTurn = (OPEN_VALUE_TURN - CLOSE_VALUE_TURN);
int diffValuePush = (OPEN_VALUE_PUSH - CLOSE_VALUE_PUSH);
int b_turn = - CLOSE_VALUE_TURN * 100 / diffValueTurn;
int b_push = - CLOSE_VALUE_PUSH * 100 / diffValuePush;
int percentTurn = 0;
int percentPush = 0;
boolean force = false;

void callback(const gripper_msg::SetGripperRequest & req,
		    gripper_msg::SetGripperResponse & res)
{
	if (req.stateTurn)
	{
		gc_turn = OPEN;
	}
	else
	{
		gc_turn = CLOSE;
	}

	if (req.stateTurn)
	{
		gc_push = OPEN;
	}
	else
	{
		gc_push = CLOSE;
	}
}

ros::ServiceServer<gripper_msg::SetGripperRequest, gripper_msg::SetGripperResponse> gripper_server("hardware/gripper_srv", &callback);
gripper_msg::GripperStatus gripperStatus_msg;
ros::Publisher gripper_pub("hardware/gripper_status", &gripperStatus_msg);

void setup()
{
	servoTurn.attach(9);  // attaches the turn servo on pin 9 to the servo object
	servoTurn.write(targetTurn); //initialize the servo

	servoPush.attach(10);  // attaches the push servo on pin 10 to the servo object
	servoPush.write(targetPush); //initialize the servo

	gc_turn = NONE;
	gc_push = NONE;
	gs_turn = OPENED;
	gs_push = OPENED;

	nh.initNode();
	nh.advertiseService(gripper_server);
	nh.advertise(gripper_pub);
}

int subFreq = 5;
int iterFreq = 0;

void loop()
{
	//updatecmd
	switch (gc_turn)
	{
	case OPEN:
		if (gs_turn == CLOSED || gs_turn == CLOSING)
		{
			gs_turn = OPENING;
			targetTurn = OPEN_VALUE_TURN;
		}
		break;
	case CLOSE:
		if (force == false && (gs_turn == OPEN || gs_turn == OPENING))
		{
			gs_turn = CLOSING;
			targetTurn = CLOSE_VALUE_TURN;
		}
		break;
	case FORCE_OPEN:
		gs_turn = OPENING;
		targetTurn = OPEN_VALUE_TURN;
		force = true;
		break;
	case ENDMVT:
		if (gs_turn == OPENING)
		{
			gs_turn = OPENED;
		}
		else if (gs_turn == CLOSING)
		{
			gs_turn = CLOSED;
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
	gc_turn = NONE;


	switch (gc_push)
	{
	case OPEN:
		if (gs_push == CLOSED || gs_push == CLOSING)
		{
			gs_push = OPENING;
			targetPush = OPEN_VALUE_PUSH;
		}
		break;
	case CLOSE:
		if (force == false && (gs_push == OPEN || gs_push == OPENING))
		{
			gs_push = CLOSING;
			targetPush = CLOSE_VALUE_PUSH;
		}
		break;
	case FORCE_OPEN:
		gs_push = OPENING;
		targetPush = OPEN_VALUE_PUSH;
		force = true;
		break;
	case ENDMVT:
		if (gs_push == OPENING)
		{
			gs_push = OPENED;
		}
		else if (gs_push == CLOSING)
		{
			gs_push = CLOSED;
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
	gc_push = NONE;

	//update state
	switch (gs_turn)
	{
	case OPENED:
		break;
	case CLOSED:
		break;
	case OPENING:
		if (currentTurn == targetTurn)
		{
			gc_turn = ENDMVT;
		}
		if (currentTurn < targetTurn)
		{
			currentTurn = currentTurn + 1;
		}
		if (currentTurn > targetTurn)
		{
			currentTurn = targetTurn;
		}
		break;
	case CLOSING:
		if (currentTurn == targetTurn)
		{
			gc_turn = ENDMVT;
		}
		if (currentTurn < targetTurn)
		{
			currentTurn = targetTurn;
		}
		if (currentTurn > targetTurn)
		{
			currentTurn = currentTurn - 1;
		}
		break;
	case FAULTY:
		break;
	default:
	case UNKOWN:
		break;
	}


	switch (gs_push)
	{
	case OPENED:
		break;
	case CLOSED:
		break;
	case OPENING:
		if (currentPush == targetPush)
		{
			gc_push = ENDMVT;
		}
		if (currentPush < targetPush)
		{
			currentPush = currentPush + 1;
		}
		if (currentPush > targetPush)
		{
			currentPush = targetPush;
		}
		break;
	case CLOSING:
		if (currentPush == targetPush)
		{
			gc_push = ENDMVT;
		}
		if (currentPush < targetPush)
		{
			currentPush = targetPush;
		}
		if (currentPush > targetPush)
		{
			currentPush = currentPush - 1;
		}
		break;
	case FAULTY:
		break;
	default:
	case UNKOWN:
		break;
	}

	servoTurn.write(currentTurn);
	servoPush.write(currentPush);

	percentTurn = currentTurn * 100 / diffValueTurn + b_turn;
	percentPush = currentPush * 100 / diffValuePush + b_push;

	if (!iterFreq)
	{
		gripperStatus_msg.timeStamp = nh.now();
		switch (gs_turn)
		{
		case OPENED:
		case CLOSING:
			gripperStatus_msg.turn = true;
			break;
		case CLOSED:
		case OPENING:
			gripperStatus_msg.turn = false;
			break;
		case FAULTY:
		default:
		case UNKOWN:
			gripperStatus_msg.turn = false;
			break;
		}
		switch (gs_push)
		{
		case OPENED:
		case CLOSING:
			gripperStatus_msg.push = true;
			break;
		case CLOSED:
		case OPENING:
			gripperStatus_msg.push = false;
			break;
		case FAULTY:
		default:
		case UNKOWN:
			gripperStatus_msg.push = false;
			break;
		}
		gripperStatus_msg.statusPercentTurn = percentTurn;
		gripperStatus_msg.statusPercentPush = percentPush;
		gripperStatus_msg.force = force;
		gripper_pub.publish(&gripperStatus_msg);
	}
	iterFreq = (++iterFreq) % subFreq;

	nh.spinOnce();
	delay(20);    

}

