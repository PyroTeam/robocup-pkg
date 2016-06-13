#include "LocaSubscriber.h"
#include "MPS.h"

LocaSubscriber::LocaSubscriber():m_machines(24)
{
	ros::NodeHandle n;
	m_sub = n.subscribe("objectDetection/landmarks",1,&LocaSubscriber::machinesCallback, this);
}

LocaSubscriber::~LocaSubscriber()
{

}

void LocaSubscriber::machinesCallback(const deplacement_msg::MachinesConstPtr &msg)
{
	ROS_INFO_ONCE("I heard the localisation publisher ");

  for (auto &it : msg->landmarks)
  {
  	m_machines[it.zone].pose   = it.pose;
  	m_machines[it.zone].isHere = true;
  	m_machines[it.zone].zone   = it.zone;
	}

  for (auto &it : m_machines)
  {
    if (it.isHere)
    {
      ROS_INFO("Machine connue en zone %d", it.zone);
    }
  }

}

void LocaSubscriber::spin()
{
  ros::spinOnce();
}
