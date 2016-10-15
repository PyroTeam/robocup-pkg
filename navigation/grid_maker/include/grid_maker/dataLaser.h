/**
 * \file         dataLaser.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-29
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef DATALASER_H
#define DATALASER_H

#include "grid_maker/map.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>

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
    bool m_receiveScan;
    ros::Subscriber m_laser_sub;
    ros::Subscriber m_grid_sub;
    ros::Publisher m_grid_pub;
    tf::TransformListener m_listener;
    tf::StampedTransform m_transform;
    std::vector<geometry_msgs::Point> m_dataLaser;
    nav_msgs::OccupancyGrid m_grid;
    sensor_msgs::LaserScan m_scan;
    Map m_map;

    void gridCallback(const nav_msgs::OccupancyGrid &grid);
    void scanCallback(const sensor_msgs::LaserScan &scan);

protected:
    ros::NodeHandle m_nh;

public:
    std::vector<geometry_msgs::Point> getDataLaser();
    void recoverDataLaser();

    DataLaser()
    : m_listener(ros::Duration(30))
    {
        m_receiveGrid = false;
        m_receiveScan = false;
        m_grid_sub = m_nh.subscribe("objectDetection/grid", 1, &DataLaser::gridCallback, this);
        m_grid_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("objectDetection/gridObstacles", 1, true);
        m_laser_sub = m_nh.subscribe("hardware/scan", 1, &DataLaser::scanCallback, this);
    }

    ~DataLaser()
    {
    }
};

#endif /* DATALASER_H */
