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
#define RADIUS_CIRCLE    0.75
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

/*void Map::drawCircle(nav_msgs::OccupancyGrid &grid, geometry_msgs::Point center)
{
    geometry_msgs::Point point;
    float i = 0;
    float x = 0;
    float y = 0;
    int cell = 0;
    while (i < 2 * M_PI)
    {
        x = RADIUS_CIRCLE * cos(i);
        y = RADIUS_CIRCLE * sin(i);
        point.x = center.x + x;
        point.y = center.y + y;
        cell = getCell(grid, point.x, point.y);
        ROS_INFO("i = %f", i);
        ROS_INFO("cell = %d", cell);
        if (grid.data[cell] == 0)
        {
            ROS_INFO("Cellule non remplie");
            grid.data[cell] = 100;
        }
        i += INCREMENT_ANGLE; // On incrémente de 5°
    }
    m_grid_pub.publish(grid);
}*/

/*void Map::fillCircle(nav_msgs::OccupancyGrid &grid, geometry_msgs::Point center)
{
    geometry_msgs::Point pointDepart;
    geometry_msgs::Point pointArrivee;
    int cpt = 1;
    float x1 = 0;
    float y1 = 0;
    int cell1 = 0;
    float j = 0;
    float x2 = 0;
    float y2 = 0;
    int cell2 = 0;

    // Pour la partie "basse" 
    float i = 1.5 * M_PI + INCREMENT_ANGLE;
    while (i < 2 * M_PI)
    {
        x1 = RADIUS_CIRCLE * sin(i);
        y1 = RADIUS_CIRCLE * cos(i);
        pointDepart.x = x1 + center.x;
        pointDepart.y = y1 + center.y;
        cell1 = getCell(grid, pointDepart.x, pointDepart.y);

        j = i - 2 * cpt * INCREMENT_ANGLE;
        x2 = RADIUS_CIRCLE * sin(j);
        y2 = RADIUS_CIRCLE * cos(j);
        pointArrivee.x = x2 + center.x;
        pointArrivee.y = y2 + center.y;
        cell2 = getCell(grid, pointArrivee.x, pointArrivee.y);

        for (int k = cell1 + 1 ; k < cell2 ; k++)
        {
            if (grid.data[k] == 0)
            {
                grid.data[k] = 100;
            }
        }
        cpt++;
        i += INCREMENT_ANGLE;
    }

    // Pour la partie "haute"
    i = 0;
    while (i < M_PI / 2)
    {
        x1 = RADIUS_CIRCLE * sin(i);
        y1 = RADIUS_CIRCLE * cos(i);
        pointDepart.x = x1;
        pointDepart.y = y1;
        cell1 = getCell(grid, pointDepart.x, pointDepart.y);

        j = i + 2 * cpt * INCREMENT_ANGLE;
        x2 = RADIUS_CIRCLE * sin(j);
        y2 = RADIUS_CIRCLE * cos(j);
        pointArrivee.x = x2;
        pointArrivee.y = y2;
        cell2 = getCell(grid, pointArrivee.x, pointArrivee.y);

        for (int k = cell1 + 1 ; k < cell2 ; k++)
        {
            if (grid.data[k] == 0)
            {
                grid.data[k] = 100;
            }
        }
        cpt--;
        i += INCREMENT_ANGLE;
    }
    m_grid_pub.publish(grid);
}*/

void Map::buildMapObstacles(nav_msgs::OccupancyGrid &grid, std::vector<geometry_msgs::Point> dataLaser)
{
    for (int i = 0 ; i < dataLaser.size() ; i++)
    {
        //drawCircle(grid, dataLaser[i]);
        //fillCircle(grid, dataLaser[i]);
        drawDisc(grid, dataLaser[i]);
    } 
}
