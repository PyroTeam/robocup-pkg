/**
 * \file        Paraboloid.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2016-05-16
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */
#include "shape/Paraboloid.h"

namespace occupancy_grid_utils {

Paraboloid::Paraboloid() : Shape()
{

}

Paraboloid::Paraboloid(const geometry_msgs::Point &p, float a, float b)
    : Shape(), m_p(p), m_a(a), m_b(b)
{

}


Paraboloid::~Paraboloid()
{

}

void Paraboloid::draw(nav_msgs::OccupancyGrid &grid, int max_value)
{

    for (unsigned int i = 0; i < grid.data.size(); ++i)
    {
        geometry_msgs::Point p =  getCellCenter(grid, i);

        double dx = (p.x-m_p.x)/m_a;
        double dy = (p.y-m_p.y)/m_b;

        double z = dx*dx + dy*dy + m_p.z;
        if (z >= max_value)
        {
            grid.data[i] = max_value;
        }
        else
        {
            grid.data[i] = int(z);
        }
    }
}


void Paraboloid::draw(deplacement_msg::OccupancyGridFloat &grid, int max_value)
{
    for (unsigned int i = 0; i < grid.data.size(); ++i)
    {
        geometry_msgs::Point p =  getCellCenter(grid, i);

        double dx = (p.x-m_p.x)/m_a;
        double dy = (p.y-m_p.y)/m_b;

        double z = dx*dx + dy*dy + m_p.z;
        if (z >= max_value)
        {
            grid.data[i] = max_value;
        }
        else
        {
            grid.data[i] = float(z);
        }
    }
}


} // namespace occupancy_grid_utils
