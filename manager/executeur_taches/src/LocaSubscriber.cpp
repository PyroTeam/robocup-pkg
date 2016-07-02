#include "LocaSubscriber.h"
#include "MPS.h"

LocaSubscriber::LocaSubscriber():m_machines(24)
{
	ros::NodeHandle n;
	m_sub = n.subscribe("objectDetection/landmarks",1,&LocaSubscriber::machinesCallback, this);
    m_explo_completed = false;
}

LocaSubscriber::~LocaSubscriber()
{

}

void LocaSubscriber::machinesCallback(const deplacement_msg::MachinesConstPtr &msg)
{
	ROS_INFO_ONCE("I can receive machine positions");

  for (auto &it : msg->landmarks)
  {
  	m_machines[it.zone-1].pose   = it.pose;
  	m_machines[it.zone-1].isHere = true;
  	m_machines[it.zone-1].zone   = it.zone;
  	m_machines[it.zone-1].orientationOk   = it.orientationOk;
    /* Added by SANDRA */
    m_machines[it.zone-1].idIn = it.idIn;
    m_machines[it.zone-1].idOut = it.idOut;
    /* Done SANDRA*/
	}

  if (msg->landmarks.size() == 12)
  {
    m_explo_completed = true;
  }
  else
  {
    m_explo_completed = false;
  }
/*
  for (auto &it : m_machines)
  {
    if (it.isHere)
    {
      ROS_INFO("Machine connue en zone %d", it.zone);
    }
  }
*/
}

void LocaSubscriber::spin()
{
  ros::spinOnce();
}
