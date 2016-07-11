/**
 * \file 		gameStateFA.h
 * \class		GameStateFA
 * \brief		classe représentant l'état du jeu
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-06-15
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef _DOCKING_GAMESTATE_FA__H_
#define _DOCKING_GAMESTATE_FA__H_

#include <ros/ros.h>

#include <comm_msg/GameState.h>

class GameStateFA
{

	public:

	GameStateFA();

	bool getAlready(){return m_already;}
	int getState(){return m_state;}
	int getPhase(){return m_phase;}
	ros::Time getTime(){return m_time;}

	/**
	 *	\brief		Callback permettant de mettre à jour les infos du topic comm_msg::GameState
	 */
	void gameStateFACallback(const comm_msg::GameState &msg);

	private:

	ros::NodeHandle m_nh;
	ros::Subscriber m_gamestateSubFA;

	bool m_already;
	int m_state;
	int m_phase;
	ros::Time m_time;

};

#endif // _DOCKING_GAMESTATE_FA__H_
