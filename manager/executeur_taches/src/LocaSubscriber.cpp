#include "LocaSubscriber.h"
#include "Machine.h"

LocaSubscriber::LocaSubscriber(){
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/landmarks",1000,&LocaSubscriber::tesCallback, this);
}

LocaSubscriber::~LocaSubscriber(){
}

void LocaSubscriber::tesCallback(const manager_msg::LandmarksConstPtr &msg)
{
	ROS_INFO("I heard the localisation publisher ");

	tab_machine = msg->landmarks;

	for(int i=0; i<tab_machine.size(); i++){
		ROS_INFO("x :landmarks[%d] = %f", i,tab_machine[i].x);
	}
}