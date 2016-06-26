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

geometry_msgs::Twist SwitchModeBehavior::generateNewSetPoint()
{
    geometry_msgs::Twist setPoint;

    switch (m_mode)
    {
    case BehaviorMode_t::FOLLOW:
        if (m_pathFollower != nullptr)
        {
            setPoint = m_pathFollower->generateNewSetPoint();
        }
        else
        {
            ROS_ERROR("In pathFollower mode, but no pathFollower assigned!");
        }
        break;
    case BehaviorMode_t::AVOID:
        if (m_avoidObstacle != nullptr)
        {
            setPoint = m_avoidObstacle->generateNewSetPoint();
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

    return setPoint;
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
            ROS_ERROR("In Avoid Obstacle mode, but no adoidObstacle assigned!");
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
