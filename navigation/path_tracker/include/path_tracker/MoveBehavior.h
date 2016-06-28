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
        m_avoidObstacle(nullptr),
        m_obstacleUnavoidable(false),
        m_maxObstacleDistance(1.0),
        m_obstacleDistance(0.0)
    {
    }

    virtual ~MoveBehavior()
    {

    }

    virtual void startTraj()
    {
        if (m_pathFollower == nullptr)
        {
            ROS_ERROR("PathFollower not initialized");
            return;
        }
        m_obstacleUnavoidable = false;
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
        if (m_avoidObstacle!=nullptr)
        {
            m_avoidObstacle->setPath(m_pathFollower->getPath());
        }
    }

    void setAvoidObstacle(std::shared_ptr<AvoidObstacle> avoidObstacle)
    {
        m_avoidObstacle = avoidObstacle;
        if (m_pathFollower !=nullptr)
        {
            m_avoidObstacle->setPath(m_pathFollower->getPath());
        }
    }

    bool isTrajectoryEnd()
    {
        return m_pathFollower->isTrajectoryEnd();
    }

    bool isObstacleUnavoidable()
    {
        return m_obstacleUnavoidable;
    }

    void setMaxObstacleDistance(float distance)
    {
        m_maxObstacleDistance = distance;
    }

    virtual float getPathError() = 0;
    virtual geometry_msgs::Twist generateNewSetpoint() = 0;
    virtual float getPercentComplete() = 0;
    virtual bool obstacleOnTrajectory(float maxDistance, float &obstacleDistance) = 0;
protected:
    ros::Subscriber m_pathSub;
    nav_msgs::Path m_path;

    std::shared_ptr<MoveState> m_moveState;
    std::shared_ptr<PathFollower> m_pathFollower;
    std::shared_ptr<AvoidObstacle> m_avoidObstacle;

    bool m_obstacleUnavoidable;
    float m_maxObstacleDistance;
    float m_obstacleDistance;

    void pathCallback(const nav_msgs::Path &path);
};

#endif /* PATH_TRACKER_MOVEBEHAVIOR_H_ */
