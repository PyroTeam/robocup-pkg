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

float TrackPathAction::calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}

void TrackPathAction::pathCallback(const pathfinder::AstarPath &path)
{
    //ROS_INFO("Chemin genere");
    bool idFound = false;
    std::list<Path>::iterator it = m_path.begin();
    while (it != m_path.end() && !idFound)
    {
        if (path.id == it->m_path_id) // on a trouvé un id correspondant
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
        if (m_path.size() == SIZE_LIST)
        {
            m_path.pop_back();
        }
        Path newPath;
        newPath.m_path_id = path.id;
        newPath.m_path_points = path.path.poses;
        m_path.push_back(newPath);
    }
    //ROS_INFO("Id = %d", path.id);
}

void TrackPathAction::odomCallback(const nav_msgs::Odometry &odom)
{
    m_odom_pose = odom.pose.pose;
}

void TrackPathAction::executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal)
{
    m_failure = false;
    m_success = false;
    m_feedback.mode = 3;
    m_as.publishFeedback(m_feedback);
    ros::Rate r(30);

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
        m_timePath = ros::Time::now();
        m_feedback.path = m_timePath;
        m_pointsPath = it->m_path_points.size();
        ROS_INFO("Id found : %d", it->m_path_id);
        m_feedback.id = it->m_path_id;
        m_as.publishFeedback(m_feedback);
        m_pathTrack.resetState();
        while (!m_failure && !m_success)
        {
            ROS_INFO("Test");
            geometry_msgs::Point pointArrivee = m_pathTrack.getPointArrivee();
            m_dataMapObstacle.calculObstacle(m_odom_pose, it->m_path_points);
            ROS_INFO("Test");
            if (m_dataMapObstacle.getObstacle() == true)
            {
                ROS_INFO("Evitement");
                m_timeAvoidance = ros::Time::now();
                m_feedback.avoidance = m_timeAvoidance;
                m_feedback.mode = 2; // Mode évitement
                m_as.publishFeedback(m_feedback);
                while (!m_avoidObstacle.failure() && !m_avoidObstacle.successAvoidance())
                {
                    m_avoidObstacle.avoid(m_dataMapObstacle.getGridObstacle(), m_odom_pose, it->m_path_points, m_as, m_feedback);
                    m_feedback.percent_complete = (it->m_path_points.size() / m_pointsPath) * 100;
                    m_as.publishFeedback(m_feedback);
                }
                if (m_avoidObstacle.failure())
                {
                    m_failure = true;
                }
            }
            else // Pas d'obstacle
            {
                m_feedback.mode = 1; // Mode suivi de chemin
                m_as.publishFeedback(m_feedback);

                ROS_INFO("Path track");
                ROS_INFO("Nb points chemin : %d", (int)it->m_path_points.size());
                ROS_INFO("debut chemin : %f %f", it->m_path_points.front().pose.position.x, it->m_path_points.front().pose.position.y);
                ROS_INFO("fin chemin : %f %f", it->m_path_points.back().pose.position.x, it->m_path_points.back().pose.position.y);

                m_pathTrack.track(it->m_path_points, m_odom_pose);
                m_dataMapObstacle.calculObstacle(m_odom_pose, it->m_path_points);
                while (!m_dataMapObstacle.getObstacle() && !m_pathTrack.success() && !m_pathTrack.failure())
                {
                    m_pathTrack.track(it->m_path_points, m_odom_pose);
                    m_feedback.percent_complete = (it->m_path_points.size() / m_pointsPath) * 100;
                    m_as.publishFeedback(m_feedback);
                    m_dataMapObstacle.calculObstacle(m_odom_pose, it->m_path_points);
                    //ROS_INFO("Obstacle : %d", m_dataMapObstacle.getObstacle());
                }
                if (m_pathTrack.success())
                {
                    m_success = true;
                }
                else if (m_pathTrack.failure())
                {
                    m_failure = true;
                }
            }
        }
        if (m_failure)
        {
            ROS_INFO("Erreur action aborted");
            m_result.result = deplacement_msg::TrackPathResult::ERROR;
            m_as.setAborted(m_result); 
        }
        if (m_success)
        {
            ROS_INFO("Action successful");
            m_result.result = deplacement_msg::TrackPathResult::FINISHED;
            m_as.setSucceeded(m_result);
        }
        m_feedback.mode = 3;
        m_as.publishFeedback(m_feedback);
    }
    r.sleep();
}
