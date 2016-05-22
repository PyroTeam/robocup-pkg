#include "final_approach/GameStateFA.h"

#include <ros/ros.h>

#include <comm_msg/GameState.h>

void GameStateFA::gameStateFACallback(const comm_msg::GameState &msg)
{
	m_already = true;
	m_state=msg.state;
	m_phase=msg.phase;
	m_time=msg.game_time;
}

GameStateFA::GameStateFA()
{
	m_state = -1;
	m_phase = -1;
	m_already = false;
	m_gamestateSubFA = m_nh.subscribe("refBoxComm/GameState", 1, &GameStateFA::gameStateFACallback, this);
}
