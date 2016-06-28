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
#include <nav_msgs/Path.h>

#include "common_utils/controller/Controller.h"
#include "common_utils/RobotPoseSubscriber.h"
enum class PathFollowerStatus_t
{
    START,
    IN_PROGRESS,
    TRAJ_END,
    ERROR
};

class PathFollower
{
public:
    PathFollower(std::shared_ptr<common_utils::Controller> controllerVel,
        std::shared_ptr<common_utils::Controller> controllerOri);

    virtual ~PathFollower()
    {

    }

    void startTraj()
    {
        m_status = PathFollowerStatus_t::START;
        m_path.poses.clear();
        m_pathSize = 0;
        m_currentSegment = 0;
    }

    bool isTrajectoryEnd()
    {
        return m_status == PathFollowerStatus_t::TRAJ_END;
    }

    float getPercentComplete()
    {
        if (m_pathSize !=0)
        {
            return float(m_currentSegment)/float(m_pathSize);
        }
        else
        {
            return 0;
        }
    }

    void setControllerVel(std::shared_ptr<common_utils::Controller> controller)
    {
        m_controllerVel = controller;
    }
    void setControllerOri(std::shared_ptr<common_utils::Controller> controller)
    {
        m_controllerOri = controller;
    }

    void setVmax(double vmax)
    {
        m_Vmax = vmax;
    }

    float getPathError()
    {
        m_pathError;
    }

    virtual geometry_msgs::Twist generateNewSetpoint() = 0;

protected:
    ros::NodeHandle m_nh;
    ros::Subscriber m_pathSub;
    nav_msgs::Path m_path;

    common_utils::RobotPoseSubscriber m_robotPose;

    std::shared_ptr<common_utils::Controller> m_controllerVel;
    std::shared_ptr<common_utils::Controller> m_controllerOri;

    PathFollowerStatus_t m_status;
    int m_pathSize;
    int m_currentSegment;

    double m_Vmax;
    float m_pathError;

    void pathCallback(const nav_msgs::Path &path);
};

#endif /* PATH_TRACKER_PATHFOLLOWER_H_ */
