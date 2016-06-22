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

#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "common_utils/controller/Controller.h"

class PathFollower
{
public:
    PathFollower(std::shared_ptr<common_utils::Controller> controllerVel,
        std::shared_ptr<common_utils::Controller> controllerOri):
            m_controllerVel(controllerVel),
            m_controllerOri(controllerOri),
            m_trajEnd(true)
    {

    }

    virtual ~PathFollower()
    {

    }

    bool isTrajectoryEnd()
    {
        return m_trajEnd;
    }


    void setControllerVel(std::shared_ptr<common_utils::Controller> controller)
    {
        m_controllerVel = controller;
    }
    void setControllerOri(std::shared_ptr<common_utils::Controller> controller)
    {
        m_controllerOri = controller;
    }

    virtual geometry_msgs::Twist generateNewSetPoint() = 0;
protected:
    std::shared_ptr<common_utils::Controller> m_controllerVel;
    std::shared_ptr<common_utils::Controller> m_controllerOri;
    bool m_trajEnd;
};

#endif /* PATH_TRACKER_PATHFOLLOWER_H_ */
