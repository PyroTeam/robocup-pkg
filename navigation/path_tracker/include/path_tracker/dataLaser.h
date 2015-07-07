/**
 * \file         dataLaser.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-29
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef DATALASER_H
#define DATALASER_H

#include "map.h"

#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"

#include <vector>
#include <cmath>

class DataLaser
{
private:
    ros::Subscriber m_grid_sub;
    ros::Publisher m_grid_pub;
    bool m_obstacle;
    float m_lengthObstacle;
    std::vector<geometry_msgs::Point> m_dataLaser;
    std::vector<geometry_msgs::Point> m_vectorObstacle;
    nav_msgs::OccupancyGrid m_grid;
    Map m_map;
    bool m_receiveGrid;

    void gridCallback(const nav_msgs::OccupancyGrid &grid);

protected:
    ros::NodeHandle m_nh;

public:
    bool getObstacle();
    float getLengthObstacle();
    std::vector<geometry_msgs::Point> getDataLaser();
    void recoverDataLaser(sensor_msgs::LaserScan laserScan);
    float calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    void calculObstacle(geometry_msgs::Pose odom, geometry_msgs::Point pointArrivee);

    DataLaser()
    {
        m_receiveGrid = false;
        m_obstacle = false;
        m_lengthObstacle = 0;
        m_grid_sub = m_nh.subscribe("/grid", 1000, &DataLaser::gridCallback, this);
        m_grid_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("gridObstacles", 1000);
    }

    ~DataLaser()
    {
    }
};

#endif /* DATALASER_H */
