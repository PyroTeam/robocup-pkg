/**
 * \file         dataLaser.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-29
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "dataLaser.h"

/* Constantes */
#define LASER_RANGE_MAX 5.5

std::vector<geometry_msgs::Point> DataLaser::getDataLaser()
{
    return m_dataLaser;
}

geometry_msgs::Point transformFrame(geometry_msgs::Point &point, tf::StampedTransform &transform)
{
    geometry_msgs::Point p;
    double yaw = tf::getYaw(transform.getRotation());
    p.x = point.x*cos(yaw) - point.y*sin(yaw) + transform.getOrigin().x();
    p.y = point.x*sin(yaw) + point.y*cos(yaw) + transform.getOrigin().y();
    return p;
}

void DataLaser::recoverDataLaser()
{
    if (!m_receiveScan)
    {
        return;
    }

    std::string tf_prefix;
    m_nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    std::vector<float> &ranges = m_scan.ranges;

    if (ranges.size() != 0)
    {
        m_dataLaser.clear();
	    // m_listener.waitForTransform(tf_prefix+"odom", m_scan.header.frame_id, ros::Time(0), ros::Duration(1));
     //    m_listener.lookupTransform(tf_prefix+"odom", m_scan.header.frame_id, ros::Time(0), m_transform);
        if (!m_listener.waitForTransform(tf_prefix+"odom", m_scan.header.frame_id, m_scan.header.stamp, ros::Duration(1)))
        {
            return;
        }
        m_listener.lookupTransform(tf_prefix+"odom", m_scan.header.frame_id, m_scan.header.stamp, m_transform);
        //ROS_INFO("Position robot : x = %f, y = %f", m_transform.getOrigin().x(), m_transform.getOrigin().y()); 
        for (int i = 0 ; i < ranges.size() ; i++)
        {
            if (ranges[i] < LASER_RANGE_MAX)
            {
                //ROS_INFO("Point %d : %f < %f", i, ranges[i], m_scan.range_max); 
                float angle = m_scan.angle_min + i * m_scan.angle_increment;
                float x = ranges[i] * cos(angle);
                float y = ranges[i] * sin(angle);
                geometry_msgs::Point point;
                point.x = x;
                point.y = y;
                point.z = 0;
                point = transformFrame(point, m_transform);
                //ROS_INFO("Point %d : x = %f, y = %f", i, point.x, point.y);
                m_dataLaser.push_back(point);
            }
        }
    }

    if (!m_receiveGrid)
    {
        return;
    }

    /*geometry_msgs::Point point;
    point.x = 1;
    point.y = 1;
    m_map.drawDisc(m_grid, point);

    point.x = 1;
    point.y = 1.5;
    m_map.drawDisc(m_grid, point);*/

    for (int i = 0 ; i < m_dataLaser.size() ; i++)
    {
        //ROS_INFO("Point %d ajoute", i);
        m_map.drawDisc(m_grid, m_dataLaser[i]);
    }

    m_grid.header.stamp = m_scan.header.stamp;
    m_grid_pub.publish(m_grid);
    //ROS_INFO("Map obstacles publiee");
}

void DataLaser::gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    //ROS_INFO("Reception map");
    m_grid = grid;
    m_receiveGrid = true;
}

void DataLaser::scanCallback(const sensor_msgs::LaserScan &scan)
{
    //ROS_INFO("Reception donnees laser");
    m_scan = scan;
    m_receiveScan = true;
}
