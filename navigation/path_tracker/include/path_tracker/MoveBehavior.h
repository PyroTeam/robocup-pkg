/**
 * \file         MoveBehavior.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-12
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef PATH_TRACKER_MOVEBEHAVIOR_H_
#define PATH_TRACKER_MOVEBEHAVIOR_H_

#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "MoveState.h"
#include "PathFollower.h"
#include "AvoidObstacle.h"

class MoveBehavior
{
public:
    MoveBehavior():
        m_moveState(nullptr),
        m_pathFollower(nullptr),
        m_avoidObstacle(nullptr)
    {
    }

    virtual ~MoveBehavior()
    {

    }

    void startTraj()
    {
        if (m_pathFollower == nullptr)
        {
            ROS_ERROR("PathFollower not initialized");
            return;
        }
        m_pathFollower->startTraj();
        //TODO init m_avoidObstacle
    }

    void setState(std::shared_ptr<MoveState> newState)
    {
        m_moveState = newState;
    }

    void setPathFollower(std::shared_ptr<PathFollower> pathFollower)
    {
        m_pathFollower = pathFollower;
    }

    void setAvoidObstacle(std::shared_ptr<AvoidObstacle> avoidObstacle)
    {
        m_avoidObstacle = avoidObstacle;
    }

    bool isTrajectoryEnd()
    {
        return m_pathFollower->isTrajectoryEnd();
    }

    virtual float getPathError() = 0;
    virtual geometry_msgs::Twist generateNewSetPoint() = 0;
    virtual float getPercentComplete() = 0;
protected:
    std::shared_ptr<MoveState> m_moveState;
    std::shared_ptr<PathFollower> m_pathFollower;
    std::shared_ptr<AvoidObstacle> m_avoidObstacle;
};

#endif /* PATH_TRACKER_MOVEBEHAVIOR_H_ */
