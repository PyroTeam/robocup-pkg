/**
 * \file         trackPathAction.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-15
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef TRACKPATHACTION_H
#define TRACKPATHACTION_H

#include <iostream>
#include <vector>
#include <list>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "trackPath.h"

#include "deplacement_msg/TrackPathAction.h"
#include "pathfinder/AstarPath.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

/* Constantes */
#define SIZE_LIST 10

//class DataLaser;
//class AvoidanceObstacle;

class TrackPathAction
{
private:
    ros::Subscriber m_path_sub;
    ros::Subscriber m_odom_sub;
    bool m_succes;

    struct Path
    {
        int m_path_id;
        std::vector<geometry_msgs::PoseStamped> m_path_points;
    };

    std::list<Path> m_path;
    geometry_msgs::Pose m_odom_pose;
    
    TrackPath m_pathTrack;
    //AvoidanceObstacle m_avoidObstacle;
    //DataLaser m_dataLaser;

    void pathCallback(const pathfinder::AstarPath &path);
    void odomCallback(const nav_msgs::Odometry &odom);

protected:
    ros::NodeHandle m_nh;
    actionlib::SimpleActionServer<deplacement_msg::TrackPathAction> m_as;
    std::string m_action_name;
    deplacement_msg::TrackPathFeedback m_feedback;
    deplacement_msg::TrackPathResult m_result;

public:
    TrackPathAction(std::string name) : m_as(m_nh, name, boost::bind(&TrackPathAction::executeCB, this, _1), false), m_action_name(name)
    {
        m_odom_sub = m_nh.subscribe("/odom", 1000, &TrackPathAction::odomCallback, this);
        m_path_sub = m_nh.subscribe("/pathFound", 1000, &TrackPathAction::pathCallback, this);
        m_succes = false;
        m_as.start();
    }

    ~TrackPathAction()
    {
    }

    void executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal);
};

#endif /*TRACKPATHACTION_H*/
