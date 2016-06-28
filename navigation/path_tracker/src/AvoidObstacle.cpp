/**
 * \file         AvoidObstacle.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-27
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "path_tracker/AvoidObstacle.h"

AvoidObstacle::AvoidObstacle():m_nh(), m_threshObstacle(0)
{
    m_gridObstacleSub = m_nh.subscribe("objectDetection/gridObstacles", 1, &AvoidObstacle::gridCallback, this);
}

AvoidObstacle::~AvoidObstacle()
{

}

void AvoidObstacle::gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    m_gridObstacle = grid;
}
