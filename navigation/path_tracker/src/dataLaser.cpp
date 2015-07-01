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

void DataLaser::recoverDataLaser(sensor_msgs::LaserScan laserScan)
{
    std::vector<float> ranges = laserScan.ranges;
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
    }
}

float DataLaser::calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

void DataLaser::calculObstacle(geometry_msgs::Pose odom, geometry_msgs::Point pointArrivee)
{
    if (m_dataLaser.size() != 0)
    {
        int i = 0;
        int cpt = 0;
        bool obstacle = false;
        while (i < m_dataLaser.size())
        {
            if (cpt == 10)
            {
                obstacle = true;
                while (i < m_dataLaser.size() && calculDistance(odom.position, m_dataLaser[i]) < 1)
                {
                    if (calculDistance(m_dataLaser[i-1], m_dataLaser[i]) < 0.5)
                    {
                        m_vectorObstacle.push_back(m_dataLaser[i]);
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else if (cpt == 0 && calculDistance(odom.position, m_dataLaser[i]) < 1)
            {
                m_vectorObstacle.push_back(m_dataLaser[i]);
                cpt++;
            }
            else if (cpt > 0 && cpt < 10 && calculDistance(odom.position, m_dataLaser[i]) < 1)
            {
                if (calculDistance(m_dataLaser[i-1], m_dataLaser[i]) < 0.5)
                {
                    m_vectorObstacle.push_back(m_dataLaser[i]);
                    cpt++;
                }
                else
                {
                    cpt = 1;
                    m_vectorObstacle.clear();
                    m_vectorObstacle.push_back(m_dataLaser[i]);
                }
            }
            else if (calculDistance(odom.position, m_dataLaser[i]) > 1)
            {
                cpt = 0;
                m_vectorObstacle.clear();
            }
            if (obstacle)
            {
                float a = (pointArrivee.y - odom.position.y)/(pointArrivee.x - odom.position.x);
                float b = pointArrivee.y - a*pointArrivee.x;
                int j = 0;
                bool found = false;
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
            }
            i++;
        }
    }
}
