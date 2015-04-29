#ifndef GAMESTATE_H
#define GAMESTATE_H

#include <ros/ros.h>
#include "comm_msg/GameState.h" //a passer apres en comm_msg/GameState.h

class GameState {

public:

    GameState();

    int getState(){return m_state;}
    int getPhase(){return m_phase;}
    ros::Time getTime(){return m_time;}
    int getPoints(){return m_points;}

    void gsCallback(const comm_msg::GameState &msg); //ici aussi

private:

    ros::NodeHandle m_nh;
    ros::Subscriber m_gamestateSub;

    int m_state;
    int m_phase;
    ros::Time m_time;
    int m_points;

};

#endif
