/**
 * \file        Line.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-19
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "shape/Line.h"

namespace occupancy_grid_utils {

Line::Line() : Shape()
{

}

Line::~Line()
{

}

void Line::build(const geometry_msgs::Pose2D &A, const geometry_msgs::Pose2D &B)
{
	//mise en equation de la droite sous la forme
	// x*cos(theta) + y*sin(theta) = rho

	geometry_msgs::Pose2D v;
	v.x = B.x - A.x;
	v.y = B.y - A.y;
	//normalise v
	double vnorm = sqrt(v.x*v.x + v.y*v.y);
	v.x = v.x/vnorm;
	v.y = v.y/vnorm;

	//a*x + b*y + c = 0 (v = (-b a))
	m_a = v.y;
	m_b = -v.x;
	m_c = - m_a * A.x - m_b * A.y;

	m_theta = atan2(-v.x, v.y);
	if (m_c >= 0)
	{
		m_rho = m_c;
		if (m_theta > 0)
		{
			m_theta = m_theta - M_PI;
		}
		else
		{
			m_theta = m_theta + M_PI;
		}

	}
	else
	{
		m_rho = -m_c;

	}
}

void Line::get2Points(geometry_msgs::Point &p1, geometry_msgs::Point &p2) const
{
	// m (rho*cos(theta), rho*sin(theta))
	geometry_msgs::Point m;
	m.x = -m_c*m_a;
	m.y = -m_c*m_b;

	// p1 = m - 5*v
	p1.x = m.x - 5 * (-m_b);
	p1.y = m.y - 5 * m_a;
	// p2 = m + 5*v
	p2.x = m.x + 5 * (-m_b);
	p2.y = m.y + 5 * m_a;
}


void Line::draw(nav_msgs::OccupancyGrid &grid, int max_value)
{

}

} // namespace occupancy_grid_utils
