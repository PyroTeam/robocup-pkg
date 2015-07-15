/**
 * \file         avoidanceObstacle.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-07-08
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef AVOIDANCEOBSTACLE_H
#define AVOIDANCEOBSTACLE_H

#include "dataMapObstacle.h"

#include "deplacement_msg/TrackPathAction.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_datatypes.h>
#include <vector>

class AvoidanceObstacle
{
private:
    bool m_failure;
    bool m_successAvoidance;
    float m_distPath;
    bool m_right;
    bool m_almostDone;
    geometry_msgs::Point m_rightObstacle;
    geometry_msgs::Point m_leftObstacle;
    geometry_msgs::Point m_pointArrival;
    geometry_msgs::Twist m_cmdVel;
    std::vector<geometry_msgs::Point> m_intermediatePath;

    ros::Publisher m_cmdVel_pub;
    ros::NodeHandle m_nh;

    DataMapObstacle m_dataMapObstacle;

    float calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    geometry_msgs::Point calculPointsPath(geometry_msgs::Point pointD, geometry_msgs::Point pointA);
    void track(geometry_msgs::Point point, geometry_msgs::Point pointSuiv, geometry_msgs::Pose odom);

public:
    void avoid(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose &odom, std::vector<geometry_msgs::PoseStamped> &path, actionlib::SimpleActionServer<deplacement_msg::TrackPathAction> &as, deplacement_msg::TrackPathFeedback &feedback);
    bool failure();
    bool successAvoidance();

    AvoidanceObstacle()
    {
        m_failure = false;
        m_successAvoidance = false;
        m_distPath = 0;
        m_right = true;
        m_almostDone = false;
        m_cmdVel_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    }
    ~AvoidanceObstacle()
    {
    }
};

#endif /* AVOIDANCEOBSTACLE_H */
