/**
 * \file         PathFollower.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-17
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef PATH_TRACKER_PATHFOLLOWER_H_
#define PATH_TRACKER_PATHFOLLOWER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class PathFollower
{
public:
    PathFollower();
    virtual ~PathFollower();

    virtual geometry_msgs::Twist generateNewSetPoint() = 0;
protected:

};

#endif /* PATH_TRACKER_PATHFOLLOWER_H_ */
