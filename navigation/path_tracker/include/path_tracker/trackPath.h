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
#include <math.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

class TrackPath
{
private:
    bool m_success;
    bool m_failure;
    bool m_stopRobot;

    geometry_msgs::Point m_pointArrivee;
    geometry_msgs::Twist m_cmdVel;
    ros::Publisher m_cmdVel_pub;

protected:
    ros::NodeHandle m_nh;

public:
    geometry_msgs::Point getPointArrivee();
    void track(std::vector<geometry_msgs::PoseStamped> points, geometry_msgs::Pose odom);
    bool comparePoints(geometry_msgs::Point point1, geometry_msgs::Point point2);
    geometry_msgs::Point closestPoint(geometry_msgs::Point segmentStart, geometry_msgs::Point segmentStop, geometry_msgs::Point point);
    bool posePath();
    bool success();
    bool failure();

    TrackPath()
    {
        m_cmdVel_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    }

    ~TrackPath()
    {
    }
};

#endif /* TRACKPATH_H */
