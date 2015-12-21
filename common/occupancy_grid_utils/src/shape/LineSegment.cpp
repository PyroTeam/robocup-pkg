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

void LineSegment::draw(nav_msgs::OccupancyGrid &grid)
{
    //implémentation de l'algorithme de Bresenham
    static const int epsilon = 1e-5;
    geometry_msgs::Point p1 = getCellAsPixelCoord(grid, m_p1);
    geometry_msgs::Point p2 = getCellAsPixelCoord(grid, m_p2);

    //std::cout << m_name << " : (" << p1.x << ", " << p1.y << "), (" << p2.x << ", " << p2.y << ")" << std::endl;

    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;
    int dx = x2 - x1;
    int dy = y2 - y1;

    if (dx != 0)
    {
        if (dx > 0)
        {
            if (dy != 0)
            {
                if (dy > 0)
                {
                    if (dx >= dy)
                    {
                        int e = dx;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (++x1 == x2)
                            {
                                break;
                            }
                            e = e - dy;
                            if (e < 0)
                            {
                                ++y1;
                                e = e + dx;
                            }
                        }
                    }
                    else
                    {
                        int e = dy;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (++y1 == y2)
                            {
                                break;
                            }
                            e = e - dx;
                            if (e < 0)
                            {
                                ++x1;
                                e = e + dy;
                            }
                        }
                    }
                }
                else // dy < 0 (et dx > 0)
                {
                    if (dx >= -dy)
                    {
                        int e = dx;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (++x1 == x2)
                            {
                                break;
                            }
                            e = e + dy;
                            if (e < 0)
                            {
                                --y1;
                                e = e + dx;
                            }
                        }
                    }
                    else
                    {
                        int e = dy;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (--y1 == y2)
                            {
                                break;
                            }
                            e = e + dx;
                            if (e > 0)
                            {
                                ++x1;
                                e = e + dy;
                            }
                        }
                    }
                }
            }
            else // dy = 0 (et dx > 0)
            {
                do {
                    setPixelCell(grid, x1, y1, 100);
                } while (++x1 != x2);
            }
        }
        else // dx < 0
        {
            if (dy != 0)
            {
                if (dy > 0)
                {
                    if (-dx >= dy)
                    {
                        int e = dx;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (--x1 == x2)
                            {
                                break;
                            }
                            e = e + dy;
                            if (e >= 0)
                            {
                                ++y1;
                                e = e + dx;
                            }
                        }
                    }
                    else
                    {
                        int e = dy;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (++y1 == y2)
                            {
                                break;
                            }
                            e = e + dx;
                            if (e <= 0)
                            {
                                --x1;
                                e = e + dy;
                            }
                        }
                    }
                }
                else // dy < 0 (et dx < 0)
                {
                    if (dx <= dy)
                    {
                        int e = dx;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (--x1 == x2)
                            {
                                break;
                            }
                            e = e - dy;
                            if (e >= 0)
                            {
                                --y1;
                                e = e + dx;
                            }
                        }
                    }
                    else
                    {
                        int e = dy;
                        dx = dx*2;
                        dy = dy*2;
                        while(1)
                        {
                            setPixelCell(grid, x1, y1, 100);
                            if (--y1 == y2)
                            {
                                break;
                            }
                            e = e - dx;
                            if (e >= 0)
                            {
                                --x1;
                                e = e + dy;
                            }
                        }
                    }
                }
            }
            else // dy = 0 (et dx < 0)
            {
                do {
                    setPixelCell(grid, x1, y1, 100);
                } while (--x1 != x2);
            }
        }
    }
    else // dx = 0
    {
        if (dy != 0)
        {
            if (dy > 0)
            {
                do {
                    setPixelCell(grid, x1, y1, 100);
                } while (++y1 != y2);
            }
            else // dy < 0 (et dx = 0)
            {
                do {
                    setPixelCell(grid, x1, y1, 100);
                } while (--y1 != y2);
            }
        }
    }
    setPixelCell(grid, x2, y2, 100);
}

} // namespace occupancy_grid_utils
