#include "LocaSubscriber.h"
#include "Machine.h"

LocaSubscriber::LocaSubscriber(){
}

LocaSubscriber::~LocaSubscriber(){
}

void LocaSubscriber::tesCallback(const manager_msg::Landmarks &msg)
{
	ROS_INFO("I heard the localisation publisher ");

	tab_machine = msg.landmarks;

	for(int i=0; i<tab_machine.size(); i++){
		ROS_INFO("x :landmarks[%d] = %f", i,tab_machine[i].x);
	}
}