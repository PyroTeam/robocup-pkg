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
        if (m_bumper)
        {
            m_beginCollision = ros::Time::now();
            m_mode = BehaviorMode_t::COLLISION;
            break;
        }
        if (m_pathFollower != nullptr)
        {
            setpoint = m_pathFollower->generateNewSetpoint();
            if(obstacleOnTrajectory(m_maxObstacleDistance, m_obstacleDistance))
            {
                ROS_INFO("Obstacle sur la trajectoire");
                m_mode = BehaviorMode_t::AVOID;
                m_obstacleUnavoidable = true;//TODO: à supprimer lorsqu'il y aura l'évitement normal
            }
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
    case BehaviorMode_t::COLLISION:
        {
            ros::Duration timeSinceCollision = ros::Time::now() - m_beginCollision;
            if (m_bumper && timeSinceCollision < ros::Duration(5.0))
            {
                break;
            }
            if (!m_bumper && timeSinceCollision >= ros::Duration(5.0))
            {
                m_mode = BehaviorMode_t::FOLLOW;
                break;
            }
            if (m_bumper && timeSinceCollision >= ros::Duration(15.0))
            {
                m_obstacleUnavoidable = true;
                break;
            }
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
    case BehaviorMode_t::COLLISION:
        error = 0.0;
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

bool SwitchModeBehavior::obstacleOnTrajectory(float maxDistance, float &obstacleDistance)
{
    int currentSegment = m_pathFollower->getCurrentSegment();
    return m_avoidObstacle->obstacleOnTrajectory(currentSegment, maxDistance, obstacleDistance);
}
