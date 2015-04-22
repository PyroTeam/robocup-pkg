#include "action.h"
#include <ros/ros.h>
#include "robot.h"

#include "manager_msg/activity.h"

void Action::tesCallback(const manager_msg::activity &msg)
{
  m_nb_robot = msg.nb_robot;
  m_state = msg.state;
  m_machine_used = msg.machine_used;
  m_nb_order = msg.nb_order;
  ROS_INFO("nb_robot: %d,state: %d,machine_used: %d,nb_order: %d",m_nb_robot,m_state,m_machine_used,m_nb_order);
}

Action::Action(){
  m_activity_sub = m_nh.subscribe("task_exec_state",1000,&Action::tesCallback,this);        
}

void Action::update_robot(Robot (&robot)[3]){
  for(int i=0;i<3;i++){
    if(m_nb_robot == i){
      robot[i].set_machine(m_machine_used);
      robot[i].set_nb_ordre(m_nb_order);
      if(m_state==manager_msg::activity::IN_PROGRESS || m_state==manager_msg::activity::ERROR)
	robot[i].set_occupe(true);
      else
	robot[i].set_occupe(false);
    }
  }
}
