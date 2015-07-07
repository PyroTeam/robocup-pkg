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

bool DataLaser::getObstacle()
{
    return m_obstacle;
}

float DataLaser::getLengthObstacle()
{
    return m_lengthObstacle;
}

std::vector<geometry_msgs::Point> DataLaser::getDataLaser()
{
    return m_dataLaser;
}

void DataLaser::recoverDataLaser(sensor_msgs::LaserScan laserScan)
{
    /*std::vector<float> ranges = laserScan.ranges;
    std::vector<geometry_msgs::Point>::iterator it = m_dataLaser.begin();
    if (ranges.size() != 0)
    {       
        for (int i = 0 ; i < ranges.size() ; i++)
        {
            float angle = laserScan.angle_min + i * laserScan.angle_increment;
            float x = ranges[i] * sin(angle);
            float y = ranges[i] * cos(angle);
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = 0;
            m_dataLaser.insert(it, point);
            it++;
        }
    }*/
    //m_map.buildMapObstacles(m_grid, m_dataLaser);

    if (!m_receiveGrid)
    {
        return;
    }
    geometry_msgs::Point point;
    point.x = 1;
    point.y = 1;
    m_map.drawDisc(m_grid, point);

    point.x = 1;
    point.y = 1.5;
    m_map.drawDisc(m_grid, point);

    m_grid_pub.publish(m_grid);
}

float DataLaser::calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

void DataLaser::calculObstacle(geometry_msgs::Pose odom, geometry_msgs::Point pointArrivee)
{
    m_obstacle = false;
    m_lengthObstacle = 0;
    if (m_vectorObstacle.size() != 0)
    {
        m_vectorObstacle.clear();
    }    
    if (m_dataLaser.size() != 0)
    {
        int i = 0;
        while (i < m_dataLaser.size() && !m_obstacle)
        {
            if (calculDistance(odom.position, m_dataLaser[i]) < 1) // Obstacle proche
            {
                while (calculDistance(odom.position, m_dataLaser[i]) < 1)
                {
                    m_vectorObstacle.push_back(m_dataLaser[i]);
                    i++;
                }
                m_lengthObstacle = calculDistance(m_vectorObstacle[0], m_vectorObstacle[i-1]);
            
                float a = (pointArrivee.y - odom.position.y)/(pointArrivee.x - odom.position.x);
                float b = pointArrivee.y - a * pointArrivee.x;
                int j = 0;
                while (j < m_vectorObstacle.size() && !m_obstacle)
                {
                    if (m_vectorObstacle[j].y == a * m_vectorObstacle[j].x + b)
                    {
                        m_obstacle = true;
                    }
                    else
                    {
                        j++;
                    }
                }
                if (j == m_vectorObstacle.size())
                {
                    m_vectorObstacle.clear();
                }
            }
            else
            {
                i++;
            }
        }
    }
}

void DataLaser::gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    m_grid = grid;
    m_receiveGrid = true;
}
