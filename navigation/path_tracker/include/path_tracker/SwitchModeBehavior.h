/**
 * \file         SwitchModeBehavior.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-12
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef PATH_TRACKER_SWITCHMODEBEHAVIOR_H_
#define PATH_TRACKER_SWITCHMODEBEHAVIOR_H_

#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "MoveState.h"
#include "MoveBehavior.h"
#include "PathFollower.h"
#include "AvoidObstacle.h"

enum class BehaviorMode_t
{
    FOLLOW,
    AVOID
};


class SwitchModeBehavior : public MoveBehavior
{
public:
    SwitchModeBehavior();
    virtual ~SwitchModeBehavior();

    virtual geometry_msgs::Twist generateNewSetpoint() override;

    BehaviorMode_t getMode()
    {
        return m_mode;
    }

    virtual float getPathError() override;
    virtual float getPercentComplete() override;
    virtual bool obstacleOnTrajectory(float maxDistance, float &obstacleDistance) override;
        
protected:
    BehaviorMode_t m_mode;
};

#endif /* PATH_TRACKER_SWITCHMODEBEHAVIOR_H_ */
