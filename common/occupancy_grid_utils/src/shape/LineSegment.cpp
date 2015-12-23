/**
 * \file        LineSegment.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-19
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "shape/LineSegment.h"

namespace occupancy_grid_utils {

LineSegment::LineSegment() : Shape()
{

}

LineSegment::LineSegment(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    : Shape(), m_p1(p1), m_p2(p2)
{

}


LineSegment::~LineSegment()
{

}

void LineSegment::draw(nav_msgs::OccupancyGrid &grid, int max_value)
{
    //implémentation de l'algorithme de Bresenham
    static const int epsilon = 1e-5;
    geometry_msgs::Point p1 = getCellAsPixelCoord(grid, m_p1);
    geometry_msgs::Point p2 = getCellAsPixelCoord(grid, m_p2);

    //std::cout << m_name << " : (" << p1.x << ", " << p1.y << "), (" << p2.x << ", " << p2.y << ")" << std::endl;

    int x0 = p1.x;
    int y0 = p1.y;
    int x1 = p2.x;
    int y1 = p2.y;

    //http://members.chello.at/easyfilter/bresenham.html

    int dx =  std::abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = -std::abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = dx+dy, e2; // error value e_xy

    for(;;){  // loop
        setPixelCell(grid, x0, y0, max_value);
        if (x0==x1 && y0==y1)
        {
            break;
        }
        e2 = 2*err;
        if (e2 >= dy)
        {
            err += dy; x0 += sx;
        } // e_xy+e_x > 0
        if (e2 <= dx)
        {
            err += dx; y0 += sy;
        } // e_xy+e_y < 0
    }
}

} // namespace occupancy_grid_utils
