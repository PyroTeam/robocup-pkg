/**
 * \file         SwitchModeBehavior.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-12
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#include "path_tracker/SwitchModeBehavior.h"

SwitchModeBehavior::SwitchModeBehavior():MoveBehavior(),m_mode(BehaviorMode_t::FOLLOW)
{

}

SwitchModeBehavior::~SwitchModeBehavior()
{

}

geometry_msgs::Twist SwitchModeBehavior::generateNewSetpoint()
{
    geometry_msgs::Twist setpoint;

    switch (m_mode)
    {
    case BehaviorMode_t::FOLLOW:
        if (m_pathFollower != nullptr)
        {
            setpoint = m_pathFollower->generateNewSetpoint();
        }
        else
        {
            ROS_ERROR("In pathFollower mode, but no pathFollower assigned!");
        }
        break;
    case BehaviorMode_t::AVOID:
        if (m_avoidObstacle != nullptr)
        {
            setpoint = m_avoidObstacle->generateNewSetpoint();
        }
        else
        {
            ROS_ERROR("In Avoid Obstacle mode, but no adoidObstacle assigned!");
        }
        break;
    default:
        ROS_ERROR("Wrong mode!");
        break;
    }

    return setpoint;
}

float SwitchModeBehavior::getPathError()
{
    float error = 0;

    switch (m_mode)
    {
    case BehaviorMode_t::FOLLOW:
        if (m_pathFollower != nullptr)
        {
            error = m_pathFollower->getPathError();
        }
        else
        {
            ROS_ERROR("In pathFollower mode, but no pathFollower assigned!");
        }
        break;
    case BehaviorMode_t::AVOID:
        if (m_avoidObstacle != nullptr)
        {
            error = m_avoidObstacle->getPathError();
        }
        else
        {
            ROS_ERROR("In Avoid Obstacle mode, but no avoidObstacle assigned!");
        }
        break;
    default:
        ROS_ERROR("Wrong mode!");
        break;
    }

    return error;
}

float SwitchModeBehavior::getPercentComplete()
{
    float percentComplete=0;
    //TODO avoidance

    if (m_pathFollower != nullptr)
    {
        percentComplete = m_pathFollower->getPercentComplete();
    }
    else
    {
        ROS_ERROR("In pathFollower mode, but no pathFollower assigned!");
    }

    return percentComplete;
}
