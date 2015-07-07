/**
 * \file         trackPathAction.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-15
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "trackPathAction.h"

void TrackPathAction::pathCallback(const pathfinder::AstarPath &path)
{
    if (m_path.size() == SIZE_LIST)
    {
        m_path.pop_back();
    }
    Path newPath;
    newPath.m_path_id = path.id;
    newPath.m_path_points = path.path.poses;
    m_path.push_back(newPath);
}

void TrackPathAction::odomCallback(const nav_msgs::Odometry &odom)
{
    m_odom_pose = odom.pose.pose;
}

void TrackPathAction::scanCallback(const sensor_msgs::LaserScan &scan)
{
    m_scan = scan;
}

void TrackPathAction::executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal)
{
    // On regarde si l'id demandé par l'action se situe dans le tableau
    bool idFound = false;
    std::list<Path>::iterator it = m_path.begin();
    while (it != m_path.end() && !idFound)
    {
        if (goal->id == it->m_path_id) // on a trouvé un id correspondant
        {
            idFound = true;
        }
        else
        {
            it++;
        }
    }

    if (!idFound)
    {
        ROS_INFO("No id found");
        m_result.result = deplacement_msg::TrackPathResult::ERROR;
        m_as.setAborted(m_result);
    }
    else /* id found */
    {
        while (!m_pathTrack.success() && !m_pathTrack.failure() /*&& !m_avoidObstacle.failure()*/)
        {
            geometry_msgs::Point pointArrivee = m_pathTrack.getPointArrivee();
            m_dataLaser.calculObstacle(m_odom_pose, pointArrivee);
            if (m_dataLaser.getObstacle() == true)
            {
                m_mode = 2; // Mode évitement
                //m_avoidObstacle.avoidance();
                m_dataLaser.calculObstacle(m_odom_pose, pointArrivee);
                while (m_dataLaser.getObstacle() == true && !m_pathTrack.success() && !m_pathTrack.failure())
                {
                    //m_avoidObstacle.avoidance();
                    /*if (m_avoidObstacle.failure())
                    {
                        break;
                    }*/
                    m_dataLaser.calculObstacle(m_odom_pose, pointArrivee);
                }
            }
            else // Pas d'obstacle
            {
                m_mode = 1; // Mode suivi de chemin
                m_pathTrack.track(it->m_path_points, m_odom_pose);
                m_dataLaser.calculObstacle(m_odom_pose, pointArrivee);
                while (m_dataLaser.getObstacle() == false && !m_pathTrack.success() && !m_pathTrack.failure())
                {
                    m_pathTrack.track(it->m_path_points,m_odom_pose);
                    if (m_pathTrack.success() || m_pathTrack.failure())
                    {
                        break;
                    }
                    m_dataLaser.calculObstacle(m_odom_pose, pointArrivee);
                }      
            }
        }
        if (m_pathTrack.failure() /*&& m_avoidObstacle.failure()*/)
        {
            ROS_INFO("Erreur action aborted");
            m_result.result = deplacement_msg::TrackPathResult::ERROR;
            m_as.setAborted(m_result); 
        }
        if (m_pathTrack.success())
        {
            ROS_INFO("Action successful");
            m_result.result = deplacement_msg::TrackPathResult::FINISHED;
            m_as.setSucceeded(m_result);
        }
        m_mode = 3;
    }
}
