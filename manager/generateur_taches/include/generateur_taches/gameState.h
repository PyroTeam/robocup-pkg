/**
 * \file 		gameState.h
 * \class		GameState
 * \brief		classe représentant l'état du jeu
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */

#ifndef GAMESTATE_H
#define GAMESTATE_H

#include <ros/ros.h>
#include "comm_msg/GameState.h" 

class GameState {

public:

    GameState();

    int getState(){return m_state;}
    int getPhase(){return m_phase;}
    ros::Time getTime(){return m_time;}
    int getPoints(){return m_points;}

/**
 *	\brief		Callback permettant de mettre à jour les infos du topic comm_msg::GameState
 */
    void gsCallback(const comm_msg::GameState &msg); 

private:

    ros::NodeHandle m_nh;
    ros::Subscriber m_gamestateSub;

    int m_state;
    int m_phase;
    ros::Time m_time;
    int m_points;

};

#endif
