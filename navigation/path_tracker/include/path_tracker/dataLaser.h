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
    bool m_receiveGrid;    
    ros::Subscriber m_grid_sub;
    ros::Publisher m_grid_pub;
    std::vector<geometry_msgs::Point> m_dataLaser;
    nav_msgs::OccupancyGrid m_grid;
    Map m_map;

    void gridCallback(const nav_msgs::OccupancyGrid &grid);

protected:
    ros::NodeHandle m_nh;

public:
    std::vector<geometry_msgs::Point> getDataLaser();
    void recoverDataLaser(sensor_msgs::LaserScan laserScan);

    DataLaser()
    {
        m_receiveGrid = false;
        m_grid_sub = m_nh.subscribe("/grid", 1000, &DataLaser::gridCallback, this);
        m_grid_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("gridObstacles", 1000);
    }

    ~DataLaser()
    {
    }
};

#endif /* DATALASER_H */
