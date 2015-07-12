/**
 * \file         dataMapObstacle.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-07-09
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "dataMapObstacle.h"

bool DataMapObstacle::getObstacle()
{
    return m_obstacle;
}

geometry_msgs::Point DataMapObstacle::getObstacleLeft()
{
    return m_obstacleLeft;
}

geometry_msgs::Point DataMapObstacle::getObstacleRight()
{
    return m_obstacleRight;
}

float DataMapObstacle::getLengthObstacle()
{
    return m_lengthObstacle;
}

float DataMapObstacle::calculDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}

geometry_msgs::Point DataMapObstacle::getPoint(int cell, const nav_msgs::OccupancyGrid &grid)
{
	float res = grid.info.resolution;
	int width = grid.info.width;
	float xO = grid.info.origin.position.x;
	float yO = grid.info.origin.position.y;
    int h = 0;
    int w = 0;
    float x = 0;
    float y = 0;
    geometry_msgs::Point point;

    do
    {
        h = (cell - w) / width;
        w++;
    }while (((cell - w) % width) != 0);

    x = w * res + xO;
    y = h * res + yO;
    point.x = x;
    point.y = y;

    return point;
}

void DataMapObstacle::getPointsMap(const nav_msgs::OccupancyGrid &grid)
{
    geometry_msgs::Point point;
    for (int i = 0 ; i < grid.info.width * grid.info.height ; i++)
    {
        if (grid.data[i] != 0)
        {
            point = getPoint(i, grid);
            m_pointsGrid.push_back(point);
        }
    }
}

void DataMapObstacle::calculObstacle(geometry_msgs::Pose odom, geometry_msgs::Point pointArrivee, float distObstacle)
{
    m_obstacle = false;
    m_lengthObstacle = 0;
    geometry_msgs::Point pointTmp;
    if (m_vectorObstacle.size() != 0)
    {
        m_vectorObstacle.clear();
    }    
    if (m_pointsGrid.size() != 0)
    {
        int i = 0;
        while (i < m_pointsGrid.size() && !m_obstacle)
        {
            if (calculDistance(odom.position, m_pointsGrid[i]) < distObstacle) // Obstacle proche
            {
                while (calculDistance(odom.position, m_pointsGrid[i]) < distObstacle)
                {
                    m_vectorObstacle.push_back(m_pointsGrid[i]);
                    i++;
                }
                m_lengthObstacle = calculDistance(m_vectorObstacle[0], m_vectorObstacle[i-1]);
                pointTmp = m_vectorObstacle[i-1];
            
                float a = (pointArrivee.y - odom.position.y)/(pointArrivee.x - odom.position.x);
                float b = pointArrivee.y - a * pointArrivee.x;
                int j = 0;
                while (j < m_vectorObstacle.size() && !m_obstacle)
                {
                    if (m_vectorObstacle[j].y == a * m_vectorObstacle[j].x + b)
                    {
                        m_obstacle = true;
                        m_obstacleLeft = m_vectorObstacle[0];
                        m_obstacleRight = pointTmp;
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

void DataMapObstacle::gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    m_grid = grid;
    m_receiveGrid = true;
}
