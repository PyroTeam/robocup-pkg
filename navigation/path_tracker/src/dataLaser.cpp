/**
 * \file         dataLaser.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-06-29
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "dataLaser.h"

std::vector<geometry_msgs::Point> DataLaser::getDataLaser()
{
    return m_dataLaser;
}

void DataLaser::recoverDataLaser()
{
    std::vector<float> ranges = m_scan.ranges;
    std::vector<geometry_msgs::Point>::iterator it = m_dataLaser.begin();
    if (ranges.size() != 0)
    {       
        for (int i = 0 ; i < ranges.size() ; i++)
        {
            float angle = m_scan.angle_min + i * m_scan.angle_increment;
            float x = ranges[i] * sin(angle);
            float y = ranges[i] * cos(angle);
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = 0;
            m_dataLaser.insert(it, point);
            it++;
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
        m_map.drawDisc(m_grid, m_dataLaser[i]);
    } 

    m_grid_pub.publish(m_grid);
}

void DataLaser::gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    m_grid = grid;
    m_receiveGrid = true;
}

void DataLaser::scanCallback(const sensor_msgs::LaserScan &scan)
{
    m_scan = scan;
}
