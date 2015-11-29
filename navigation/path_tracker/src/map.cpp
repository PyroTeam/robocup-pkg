/**
 * \file         map.cpp
 *
 * \brief
 *
 * \author       Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-07-01
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "map.h"

/* Constantes */
#define RADIUS_CIRCLE    0.25
#define EPSILON          0.000001

int Map::getCell(const nav_msgs::OccupancyGrid &grid, float x, float y)
{
	float res = grid.info.resolution;
	int width = grid.info.width;
	float xO = grid.info.origin.position.x;
	float yO = grid.info.origin.position.y;
	int hCell = 0;
	int wCell = 0;
	int cell = 0;

	hCell = round((y - yO) / res);
	wCell = round((x - xO) / res);
	cell = hCell * width + wCell;

    /*ROS_INFO("x = %f", x);
    ROS_INFO("y = %f", y);
    ROS_INFO("x0 = %f", xO);
    ROS_INFO("y0 = %f", yO);
    ROS_INFO("hCell = %d", hCell);
    ROS_INFO("wCell = %d", wCell);
    ROS_INFO("cell = %d", cell);
    ROS_INFO("res = %f", res);
    ROS_INFO("width = %d", width);*/

	return cell;
}

void Map::drawDisc(nav_msgs::OccupancyGrid &grid, geometry_msgs::Point center)
{
    geometry_msgs::Point leftHigh;
    leftHigh.x = center.x + RADIUS_CIRCLE;
    leftHigh.y = center.y + RADIUS_CIRCLE;

    geometry_msgs::Point rightLow;
    rightLow.x = center.x - RADIUS_CIRCLE;
    rightLow.y = center.y - RADIUS_CIRCLE;

    geometry_msgs::Point point;
    point.x = leftHigh.x;
    point.y = leftHigh.y;
    int cell = getCell(grid, point.x, point.y);
    int cpt = 0;
    while (point.x >= rightLow.x-EPSILON)
    {
        while (point.y >= rightLow.y-EPSILON)
        {
            float x2 = (point.x - center.x)*(point.x - center.x);
            float y2 = (point.y - center.y)*(point.y - center.y);
            if (x2 + y2 <= RADIUS_CIRCLE * RADIUS_CIRCLE)
            {
                cell = getCell(grid, point.x, point.y);
                if (grid.data[cell] != 100)
                {
                    grid.data[cell] = 100;
                }
            }
            point.y -= grid.info.resolution;
        }
        point.y = leftHigh.y;
        point.x -= grid.info.resolution;
    }
}
