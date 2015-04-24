#include "etatdujeu.h"
#include <ros/ros.h>

#include "comm_msg/GameState.h" //a passer apres en comm_msg/GameState.h

void Etatdujeu::gsCallback(const manager_msg::GameState &msg) //ici aussi
{
  m_etat=msg.state;
  m_phase=msg.phase;
  m_temps=msg.game_time;
  m_points=msg.points;
}

Etatdujeu::Etatdujeu(){
  ROS_INFO("test");
  m_gamestate_sub = m_nh.subscribe("game_state",1000,&Etatdujeu::gsCallback,this);        
}

