#include "gameState.h"
#include <ros/ros.h>

#include "comm_msg/GameState.h" //a passer apres en comm_msg/GameState.h

void GameState::gsCallback(const comm_msg::GameState &msg) //ici aussi
{
	m_state=msg.state;
	m_phase=msg.phase;
	m_time=msg.game_time;
	m_points=msg.points;
}

GameState::GameState(){
	//ROS_INFO("test");
	m_gamestateSub = m_nh.subscribe("refBoxComm/GameState",1000,&GameState::gsCallback,this);
}
