/**
 * \file
 *
 * \see			common_utils/Zone.h
 *
 * \author      Thomas Danel (th.danel@gmail.com)
 * \date        2016-07-26
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "common_utils/Zone.h"
#include <common_utils/types.h>

namespace common_utils {

Zone::Zone(int number)
: m_num(number)
, m_state(3)
, m_width(2.0)
, m_height(1.5)
, m_in_sight(false)
, m_color(CYAN)
{
    this->computeCenter();
}

Zone::~Zone(void)
{

}

void Zone::num(int num)
{
    m_num = num;
}

void Zone::state(int state)
{
    m_state = state;
}

void Zone::color(int color)
{
    m_color = color;
}

void Zone::width(double width)
{
    m_width = width;
}

void Zone::height(double height)
{
    m_height = height;
}

void Zone::inSight(bool inSight)
{
    m_in_sight = inSight;
}

void Zone::center(geometry_msgs::Point center)
{
    m_center = center;
}

void Zone::center(double x, double y)
{
    m_center.x = x;
    m_center.y = y;
    m_center.z = 0.0;
}

void Zone::computeCenter()
{
    bool leftSide = m_num > 12;
    geometry_msgs::Point tmpCenter;

    if (m_num < 1 || m_num > 24)
    {
        tmpCenter.x = 0;
        tmpCenter.y = 0;
    }

    int tmpZone = (leftSide)? m_num-12 : m_num;

    tmpCenter.x = ((tmpZone-1)/4)*m_width + m_width/2;
    tmpCenter.y = ((tmpZone-1)%4)*m_height + m_height/2;
    if (leftSide)
    {
        tmpCenter.x *= -1;
    }
    tmpCenter.z = 0.0;

    this->center(tmpCenter);
}

bool Zone::center(int zone, double &x, double &y)
{
    if (zone = m_num)
    {
        geometry_msgs::Point tmp = this->center();
        x = tmp.x;
        y = tmp.y;
        return true;
    }
    return false;
}

bool Zone::isInside(const geometry_msgs::Pose2D &m, float threshold)
{
    if ((std::abs(m_center.x - m.x) <= (m_width  - threshold)/2) &&
        (std::abs(m_center.y - m.y) <= (m_height - threshold)/2))
    {
      return true;
    }
    else
    {
      return false;
    }
}

int getArea(const geometry_msgs::Pose2D &m)
{
    for (int i = 1; i <= 24; i++)
    {
        Zone area(i);

        if (area.isInside(m))
        {
            return i;
        }
    }

    return 0;
}

std::list<Zone> buildZones()
{
    std::list<Zone> list;
    for (int i = 1; i <= 24; i++)
    {
        Zone area(i);
        list.push_back(area);
    }
    return list;
}

} // namespace common_utils
