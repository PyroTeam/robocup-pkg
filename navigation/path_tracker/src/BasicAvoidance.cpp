/**
 * \file         BasicAvoidance.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-27
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */
#include "path_tracker/BasicAvoidance.h"
#include "geometry_utils/geometry_utils.h"
#include "occupancy_grid_utils/occupancy_grid_utils.h"

BasicAvoidance::BasicAvoidance():AvoidObstacle()
{
}

BasicAvoidance::~BasicAvoidance()
{

}


geometry_msgs::Twist BasicAvoidance::generateNewSetpoint()
{
    geometry_msgs::Twist twist;
    //TODO:
    return twist;
}

float BasicAvoidance::getPathError()
{
    float error = 0.0;
    //TODO:
    return error;
}

bool BasicAvoidance::obstacleOnTrajectory(int currentSegment, float maxDistance, float &obstacleDistance)
{
    bool obstacleOnTraj = false;
    //parcourir le chemin sur une distance donnée
    int sizePath = m_path->poses.size();
    int i=currentSegment + 1;
    float currentDistance = 0;

    while (currentDistance <= maxDistance
        && !obstacleOnTraj
        && i < sizePath)
    {
        currentDistance += geometry_utils::distance(m_path->poses[i-1].pose, m_path->poses[i].pose);

        if (occupancy_grid_utils::getCellValue(m_gridObstacle, m_path->poses[i].pose.position) > m_threshObstacle)
        {
            obstacleOnTraj = true;
            obstacleDistance = currentDistance;
        }
        //si l'element de la grille est non nulle alors il y a un obstacle
        ++i;
    }

    return obstacleOnTraj;
}
