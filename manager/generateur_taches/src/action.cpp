#include "action.h"
#include <ros/ros.h>
#include "robot.h"

#include "manager_msg/activity.h"

void Action::tesCallback(const manager_msg::activity &msg)
{
	m_nbRobot = msg.nb_robot;
	m_state = msg.state;
	m_usedMachine = msg.machine_used;
	m_nbOrder = msg.nb_order;
	ROS_DEBUG("nb_robot: %d,state: %d,machine_used: %d,nb_order: %d",m_nbRobot,m_state,m_usedMachine,m_nbOrder);
}

Action::Action(){
	m_activitySub = m_nh.subscribe("manager/task_exec_state",1000,&Action::tesCallback,this);
}

void Action::updateRobot(Robot (&robot)[3]){
	for(int i=0;i<3;i++)
	{
		if(m_nbRobot == i)
		{
			robot[i].setMachine(m_usedMachine);
			robot[i].setNbOrder(m_nbOrder);
			if(m_state==manager_msg::activity::IN_PROGRESS || m_state==manager_msg::activity::ERROR)
			{
				robot[i].setBusy(true);
			}
			else
			{
				robot[i].setBusy(false);
			}
		}
	}
}
