/**
 * \file         PathFollower.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-23
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#include "path_tracker/PathFollower.h"


PathFollower::PathFollower(std::shared_ptr<common_utils::Controller> controllerVel,
    std::shared_ptr<common_utils::Controller> controllerOri):
        m_robotPose("map"),
        m_controllerVel(controllerVel),
        m_controllerOri(controllerOri),
        m_status(PathFollowerStatus_t::TRAJ_END)
{
    m_pathSub = m_nh.subscribe("navigation/pathSmooth",1,  &PathFollower::pathCallback, this);
}


void PathFollower::pathCallback(const nav_msgs::Path &path)
{
    if(m_status == PathFollowerStatus_t::START)
    {
        m_path = path;
        m_pathSize = path.poses.size();
        m_status = PathFollowerStatus_t::IN_PROGRESS;
    }
}
