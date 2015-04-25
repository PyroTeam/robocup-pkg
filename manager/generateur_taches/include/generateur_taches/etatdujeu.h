#ifndef ETATDUJEU_H
#define ETATDUJEU_H

#include <ros/ros.h>
#include "comm_msg/GameState.h" //a passer apres en comm_msg/GameState.h

class Etatdujeu {

public:

    Etatdujeu();

    int get_etat(){return m_etat;}
    int get_phase(){return m_phase;}
    ros::Time get_temps(){return m_temps;}
    int get_points(){return m_points;}

    void gsCallback(const comm_msg::GameState &msg); //ici aussi

private:

    ros::NodeHandle m_nh;
    ros::Subscriber m_gamestate_sub;

    int m_etat;
    int m_phase;
    ros::Time m_temps;
    int m_points;

};

#endif
