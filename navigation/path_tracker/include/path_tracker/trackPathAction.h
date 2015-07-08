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
#include "dataLaser.h"

#include "deplacement_msg/TrackPathAction.h"
#include "pathfinder/AstarPath.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

/* Constantes */
#define SIZE_LIST 10

//class AvoidanceObstacle;

class TrackPathAction
{
private:
    ros::Subscriber m_path_sub;
    ros::Subscriber m_odom_sub;
    //ros::Subscriber m_laser_sub;
    bool m_succes;
    int m_mode;

    struct Path
    {
        int m_path_id;
        std::vector<geometry_msgs::PoseStamped> m_path_points;
    };

    std::list<Path> m_path;
    geometry_msgs::Pose m_odom_pose;
    //sensor_msgs::LaserScan m_scan;
    
    TrackPath m_pathTrack;
    //AvoidanceObstacle m_avoidObstacle;
    //DataLaser m_dataLaser;

    void pathCallback(const pathfinder::AstarPath &path);
    void odomCallback(const nav_msgs::Odometry &odom);
    //void scanCallback(const sensor_msgs::LaserScan &scan);

    ros::NodeHandle m_nh;
    actionlib::SimpleActionServer<deplacement_msg::TrackPathAction> m_as;
    std::string m_action_name;
    deplacement_msg::TrackPathFeedback m_feedback;
    deplacement_msg::TrackPathResult m_result;

public:
    TrackPathAction(std::string name) : m_as(m_nh, name, boost::bind(&TrackPathAction::executeCB, this, _1), false), m_action_name(name)
    {
        m_mode = 3;
        m_odom_sub = m_nh.subscribe("/odom", 1000, &TrackPathAction::odomCallback, this);
        m_path_sub = m_nh.subscribe("/pathFound", 1000, &TrackPathAction::pathCallback, this);
        //m_laser_sub = m_nh.subscribe("/scan", 1000, &TrackPathAction::scanCallback, this);
        m_succes = false;
        m_as.start();
    }

    ~TrackPathAction()
    {
    }

    void executeCB(const deplacement_msg::TrackPathGoalConstPtr &goal);
};

#endif /*TRACKPATHACTION_H*/
