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
#include "trackPath.h"

#include "deplacement_msg/TrackPathAction.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

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
    bool m_almostDone;
    geometry_msgs::Point m_rightObstacle;
    geometry_msgs::Point m_leftObstacle;
    geometry_msgs::Point m_pointArrival;
    geometry_msgs::Twist m_cmdVel;
    nav_msgs::Path m_path;
    std::vector<geometry_msgs::PoseStamped> m_intermediatePath;

    ros::Publisher m_cmdVel_pub;
    ros::Publisher m_path_pub;
    ros::NodeHandle m_nh;

    DataMapObstacle m_dataMapObstacle;
    TrackPath m_trackPath;

    float calculDistance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);
    geometry_msgs::Point calculPointsPath(const geometry_msgs::Point &pointD, const geometry_msgs::Point &pointA);
    geometry_msgs::Point getPointAwayFromObstacle(const std::vector<geometry_msgs::Point> &vectorObstacle, const geometry_msgs::Point &odom);
    //float normaliseAngle(float angle);
    //void track(geometry_msgs::Point point, geometry_msgs::Point pointSuiv, geometry_msgs::Pose odom);

public:
    void avoid(const nav_msgs::OccupancyGrid &grid, const geometry_msgs::Pose &odom, std::vector<geometry_msgs::PoseStamped> &path, actionlib::SimpleActionServer<deplacement_msg::TrackPathAction> &as, deplacement_msg::TrackPathFeedback &feedback);
    void resetMode();
    bool failure();
    bool successAvoidance();

    AvoidanceObstacle()
    {
        m_failure = false;
        m_successAvoidance = false;
        m_distPath = 0;
        m_almostDone = false;
        m_cmdVel_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        m_path_pub = m_nh.advertise<nav_msgs::Path>("intermediatePath", 1000, true);
    }
    ~AvoidanceObstacle()
    {
    }
};

#endif /* AVOIDANCEOBSTACLE_H */
