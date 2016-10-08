/** 
* \file         fakeRobotino.cpp
*
* \brief        noeud de test simulant certaines fonctions, 
*               non disponible actuellement sur le robotino
*
* \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
* \date         2015-03-17
* \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
* \version
*/

#include "fakeRobotino.h"

FakeRobotino::FakeRobotino():m_gripperState(false),m_gripperStateRequest(false)
{    
    m_timeGripperChangeState = ros::Time::now();

    m_gripperStatus_pub = m_nh.advertise<manager_msg::GripperStatus>("fakeRobotino/gripper", 1000);
    m_setGripper_srv = m_nh.advertiseService("fakeRobotino/setGripper", &FakeRobotino::SetGripperSrv, this);

}


FakeRobotino::~FakeRobotino()
{

}


bool FakeRobotino::SetGripperSrv(manager_msg::SetGripper::Request &req,
                                 manager_msg::SetGripper::Response &res)
{
    ros::Duration incTime;
    if (req.state)
    {
        ROS_INFO("Request : OPEN gripper...");
        if (req.state == m_gripperState)
        {
            ROS_INFO("Gripper already opened!!");
            return false;
        }
        incTime = ros::Duration(5.0);
    }
    else
    {
        ROS_INFO("Request : CLOSE gripper...");
        if (req.state == m_gripperState)
        {
            ROS_INFO("Gripper already closed!!");
            return false;
        }
        incTime = ros::Duration(3.0);
    }
    
    m_gripperStateRequest = req.state;
    m_timeGripperChangeState = ros::Time::now() + incTime;
    

    return true;
}

void FakeRobotino::update()
{
    updateGripper();

}

void FakeRobotino::updateGripper()
{
    if (m_gripperStateRequest != m_gripperState)
    {
        //update gripper
        if (ros::Time::now() > m_timeGripperChangeState)
        {
            if (m_gripperStateRequest)
            {
                ROS_INFO("Done : OPEN gripper...");
            }
            else
            {
                ROS_INFO("Done : CLOSE gripper...");
            }
            m_gripperState = m_gripperStateRequest;
        }
    }
    
    //publish
    manager_msg::GripperStatus gs;
    gs.timeStamp = ros::Time::now();
    gs.state = m_gripperState;
    m_gripperStatus_pub.publish(gs);
}


