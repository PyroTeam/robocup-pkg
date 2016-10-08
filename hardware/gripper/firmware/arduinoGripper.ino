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

Servo myservo;  // create servo object to control a servo 

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

GripperCommand gc;
GripperState gs;

#define OPEN_VALUE		150
#define CLOSE_VALUE		65
int target = OPEN_VALUE;
int current = OPEN_VALUE;

int diffValue = (OPEN_VALUE - CLOSE_VALUE);
int b = - CLOSE_VALUE * 100 / diffValue;
int percent = 0;
boolean force = false;

void callback(const gripper_msg::SetGripperRequest & req,
		      gripper_msg::SetGripperResponse & res)
{
	if (req.state)
	{
		gc = OPEN;
	}
	else
	{
		gc = CLOSE;
	}
}

ros::ServiceServer<gripper_msg::SetGripperRequest, gripper_msg::SetGripperResponse> gripper_server("hardware/gripper_srv", &callback);
gripper_msg::GripperStatus gripperStatus_msg;
ros::Publisher gripper_pub("hardware/gripper_status", &gripperStatus_msg);

void setup()
{
	myservo.attach(9);  // attaches the servo on pin 9 to the servo object
	myservo.write(target); //initialize the servo

	gc = NONE;
	gs = OPENED;

	nh.initNode();
	nh.advertiseService(gripper_server);
	nh.advertise(gripper_pub);
}

int subFreq = 5;
int iterFreq = 0;

void loop()
{
	//updatecmd
	switch (gc)
	{
	case OPEN:
		if (gs == CLOSED || gs == CLOSING)
		{
			gs = OPENING;
			target = OPEN_VALUE;
		}
		break;
	case CLOSE:
		if (force == false && (gs == OPEN || gs == OPENING))
		{
			gs = CLOSING;
			target = CLOSE_VALUE;
		}
		break;
	case FORCE_OPEN:
		gs = OPENING;
		target = OPEN_VALUE;
		force = true;
		break;
	case ENDMVT:
		if (gs == OPENING)
		{
			gs = OPENED;
		}
		else if (gs == CLOSING)
		{
			gs = CLOSED;
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
	gc = NONE;

	//update state
	switch (gs)
	{
	case OPENED:
		break;
	case CLOSED:
		break;
	case OPENING:
		if (current == target)
		{
			gc = ENDMVT;
		}
		if (current < target)
		{
			current = current + 1;
		}
		if (current > target)
		{
			current = target;
		}
		break;
	case CLOSING:
		if (current == target)
		{
			gc = ENDMVT;
		}
		if (current < target)
		{
			current = target;
		}
		if (current > target)
		{
			current = current - 1;
		}
		break;
	case FAULTY:
		break;
	default:
	case UNKOWN:
		break;
	}

	myservo.write(current);

	percent = current * 100 / diffValue + b;

	if (!iterFreq)
	{
		gripperStatus_msg.timeStamp = nh.now();
		switch (gs)
		{
		case OPENED:
		case CLOSING:
			gripperStatus_msg.state = true;
			break;
		case CLOSED:
		case OPENING:
			gripperStatus_msg.state = false;
			break;
		case FAULTY:
		default:
		case UNKOWN:
			gripperStatus_msg.state = false;
			break;
		}
		gripperStatus_msg.statusPercent = percent;
		gripperStatus_msg.force = force;
		gripper_pub.publish(&gripperStatus_msg);
	}
	iterFreq = (++iterFreq) % subFreq;

	nh.spinOnce();
	delay(20);

}

