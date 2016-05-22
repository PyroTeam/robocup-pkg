/**
 * \file        Circle.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-05
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "shape/Circle.h"

namespace occupancy_grid_utils {

Circle::Circle() : Shape(), m_radius(0.0)
{
    m_center.x = 0.0;
    m_center.y = 0.0;
    m_center.z = 0.0;
}

Circle::Circle(const geometry_msgs::Point &center, float radius):
     Shape(),
     m_center(center),
     m_radius(radius)
{

}

Circle::~Circle()
{

}

void Circle::draw(nav_msgs::OccupancyGrid &grid, int max_value)
{
    // Code issu de l'ancien GridMaker Obstacle de l'evitement d'obstacles

    const int Epsilon = 0.000001;
    const float radius2 = m_radius*m_radius;

    geometry_msgs::Point leftHigh;
    leftHigh.x = m_center.x + m_radius;
    leftHigh.y = m_center.y + m_radius;

    geometry_msgs::Point rightLow;
    rightLow.x = m_center.x - m_radius;
    rightLow.y = m_center.y - m_radius;

    geometry_msgs::Point point;
    point.x = leftHigh.x;
    point.y = leftHigh.y;
    int cell = getCell(grid, point.x, point.y);
    int cpt = 0;
    while (point.x >= rightLow.x-Epsilon)
    {
        while (point.y >= rightLow.y-Epsilon)
        {
            float x2 = (point.x - m_center.x)*(point.x - m_center.x);
            float y2 = (point.y - m_center.y)*(point.y - m_center.y);
            if (x2 + y2 <= radius2)
            {
                int cellValue = getCellValue(grid, point.x, point.y);
                setCell(grid, point.x, point.y, std::max(max_value, cellValue));
            }
            point.y -= grid.info.resolution;
        }
        point.y = leftHigh.y;
        point.x -= grid.info.resolution;
    }
}

void Circle::draw(deplacement_msg::OccupancyGridFloat &grid, int max_value)
{

}


} // namespace occupancy_grid_utils
