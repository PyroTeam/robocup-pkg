/**
 * \file         map.h
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-07-01
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>

#include <cmath>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"

class Map
{

public:
    int getCell(const nav_msgs::OccupancyGrid &grid, float x, float y);
    void drawDisc(nav_msgs::OccupancyGrid &grid, geometry_msgs::Point center);

    Map()
    {
    }

    ~Map()
    {
    }

};

#endif /* MAP_H */
