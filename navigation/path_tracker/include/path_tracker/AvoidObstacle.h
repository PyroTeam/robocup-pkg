/**
 * \file         AvoidObstacle.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-17
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef PATH_TRACKER_AVOIDOBSTACLE_H_
#define PATH_TRACKER_AVOIDOBSTACLE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

class AvoidObstacle
{
public:
    AvoidObstacle();
    virtual ~AvoidObstacle();

    virtual geometry_msgs::Twist generateNewSetpoint() = 0;
    virtual float getPathError() = 0;
    virtual bool obstacleOnTrajectory(int currentSegment, float maxDistance, float &obstacleDistance) = 0;
    void setPath(nav_msgs::Path *path)
    {
        m_path = path;
    }

    void setThreshObstacle(float thresh)
    {
        m_threshObstacle = thresh;
    }

protected:
    ros::NodeHandle m_nh;
    ros::Subscriber m_gridObstacleSub;
    nav_msgs::OccupancyGrid m_gridObstacle;
    nav_msgs::Path *m_path;
    float m_threshObstacle;
    void gridCallback(const nav_msgs::OccupancyGrid &grid);
};

#endif /* PATH_TRACKER_AVOIDOBSTACLE_H_ */
