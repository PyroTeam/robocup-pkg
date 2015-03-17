/** 
* \file         fakeRobotino.h
*
* \brief        noeud de test simulant certaines fonctions, 
*               non disponible actuellement sur le robotino
*
* \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
* \date         2015-03-17
* \copyright    PyroTeam, Polytech-Lille
* \license
* \version
*/
#ifndef FAKEROBOTINO_H_
#define FAKEROBOTINO_H_

#include <ros/ros.h>

#include "manager_msg/GripperStatus.h"
#include "manager_msg/SetGripper.h"

class FakeRobotino
{
public:
    FakeRobotino();
    virtual ~FakeRobotino();

    void update();

private:
    ros::NodeHandle m_nh;

    ros::Publisher m_gripperStatus_pub;
    ros::ServiceServer m_setGripper_srv;

    bool SetGripperSrv(manager_msg::SetGripper::Request &req,
                       manager_msg::SetGripper::Response &res);
                       
                       
    bool m_gripperState;
    bool m_gripperStateRequest;
    ros::Time m_timeGripperChangeState;
    
    void updateGripper();
    
};

#endif /* FAKEROBOTINO_H_ */

