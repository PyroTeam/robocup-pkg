/**
 * \file         BasicFollower.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-17
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef PATH_TRACKER_BASICFOLLOWER_H_
#define PATH_TRACKER_BASICFOLLOWER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "common_utils/RateLimiter.h"
#include "PathFollower.h"

class BasicFollower : public PathFollower
{
public:
    BasicFollower(std::shared_ptr<common_utils::Controller> controllerVel,
        std::shared_ptr<common_utils::Controller> controllerOri):
        PathFollower(controllerVel, controllerOri),
        m_speedLimiter(-1.0, 0.20, 1/10.0)
    {

    }
    virtual ~BasicFollower()
    {

    }

    void setSpeedRateLimits(double lowLimit, double highLimit, double T)
    {
        common_utils::RateLimiter r(lowLimit, highLimit, T);
        m_speedLimiter = r;
    }

    virtual geometry_msgs::Twist generateNewSetpoint() override;
protected:
    common_utils::RateLimiter m_speedLimiter;
};

#endif /* PATH_TRACKER_BASICFOLLOWER_H_ */
