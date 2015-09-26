/**
 * \file         trackPath.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-18
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef TRACKPATH_H
#define TRACKPATH_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"


class TrackPath
{
private:
    bool m_success;
    bool m_failure;
    bool m_stopRobot;

    geometry_msgs::Pose m_odom_pose;
    geometry_msgs::Point m_pointArrivee;
    geometry_msgs::Twist m_cmdVel;
    ros::Publisher m_cmdVel_pub;

    ros::NodeHandle m_nh;

public:
    geometry_msgs::Point getPointArrivee();
    float normaliseAngle(float angle);
    void track(std::vector<geometry_msgs::PoseStamped> &points, const geometry_msgs::Pose &odom);
    bool compareFloat(float x, float y);
    bool comparePoints(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);
    geometry_msgs::Point closestPoint(const geometry_msgs::Point &segmentStart, const geometry_msgs::Point &segmentStop, const geometry_msgs::Point &point);
    bool posePath();
    bool success();
    bool failure();
    void resetState();

    TrackPath()
    {
        m_cmdVel_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        m_success = false;
        m_failure = false;
    }

    ~TrackPath()
    {
    }
};

#endif /* TRACKPATH_H */
