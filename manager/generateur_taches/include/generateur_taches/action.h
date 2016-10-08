/**
 * \file 		action.h
 * \class		Action
 * \brief		classe représentant les infos collectées du topic manager_msg::activity
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef ACTION_H
#define ACTION_H

#include <ros/ros.h>
#include "robot.h"
#include "manager_msg/activity.h"

class Action {

public:

	Action();
    
    int getNbRobot(){return m_nbRobot;}
    int getState(){return m_state;}
    int getUsedMachine(){return m_usedMachine;}
    int getNbOrder(){return m_nbOrder;}
/**
 *	\brief		Callback qui met à jour les infos reçues du topic manager_msg::activity
 */    
    void tesCallback(const manager_msg::activity &msg);
    
/**
 *	\brief		met à jour l'état des robots grâce au topic manager_msg::activity
 */
    void updateRobot(Robot (&robot)[3]);


private:
    
    ros::NodeHandle m_nh;
    ros::Subscriber m_activitySub;
    
    int m_nbRobot;
    int m_state;
    int m_usedMachine;
    int m_nbOrder;
    
};

#endif
