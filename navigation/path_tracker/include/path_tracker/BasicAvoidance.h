/**
 * \file         BasicAvoidance.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-27
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef PATH_TRACKER_BASICAVOIDANCE_H_
#define PATH_TRACKER_BASICAVOIDANCE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Path.h"
#include "AvoidObstacle.h"

class BasicAvoidance : public AvoidObstacle
{
public:
    BasicAvoidance();
    virtual ~BasicAvoidance();

    virtual geometry_msgs::Twist generateNewSetpoint() override;
    virtual float getPathError() override;
    virtual bool obstacleOnTrajectory(int currentSegment, float maxDistance, float &obstacleDistance) override;

protected:

};

#endif /* PATH_TRACKER_BASICAVOIDANCE_H_ */
