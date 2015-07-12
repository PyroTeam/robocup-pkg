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

#include "dataMapObstacle.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <vector>

class AvoidanceObstacle
{
private:
    bool m_failure;
    bool m_successAvoidance;
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
    void track(geometry_msgs::Point point, geometry_msgs::Point pointSuiv, nav_msgs::Odometry odom);

public:
    void avoid(const nav_msgs::OccupancyGrid &grid, const nav_msgs::Odometry &odom, std::vector<geometry_msgs::PoseStamped> &path);
    bool failure();

    AvoidanceObstacle()
    {
        m_failure = false;
        m_successAvoidance = false;
        m_right = true;
        m_almostDone = false;
        m_cmdVel_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    }
    ~AvoidanceObstacle()
    {
    }
};
