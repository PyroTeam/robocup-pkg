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

#include "PathFollower.h"

class BasicFollower : public PathFollower
{
public:
    BasicFollower(std::shared_ptr<common_utils::Controller> controllerVel,
        std::shared_ptr<common_utils::Controller> controllerOri):
        PathFollower(controllerVel, controllerOri)
    {

    }
    virtual ~BasicFollower()
    {

    }


    virtual geometry_msgs::Twist generateNewSetPoint() override
    {
        geometry_msgs::Twist twist;
        //TODO:
        twist.linear.x = m_controllerVel->update(5);
        return twist;

    }
protected:

};

#endif /* PATH_TRACKER_BASICFOLLOWER_H_ */
