/**
 * \file        Line.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-19
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_LINE_H_
#define OCCUPANCY_GRID_UTILS_LINE_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "Shape.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_utils {

class Line : public Shape
{
public:
    Line();
    ~Line();

    inline double getAngle() const;
	inline double getSlope() const;

	inline void set(double theta, double rho);
	inline void get(double &theta, double &rho) const;
	inline void getEquation(double &a, double &b, double &c) const;
	void build(const geometry_msgs::Pose2D &A, const geometry_msgs::Pose2D &B);

	void get2Points(geometry_msgs::Point &p1, geometry_msgs::Point &p2) const;

    virtual void draw(nav_msgs::OccupancyGrid &grid, int max_value=100) override;
private:
	// equation de la droite x*cos(theta) + y*sin(theta) = rho
	double m_theta;
	double m_rho;

	// equation de la droite a*x + b*y + c = 0
	double m_a, m_b, m_c;
};


inline double Line::getAngle() const
{
	return m_theta;
}

inline double Line::getSlope() const
{
	return tan(m_theta);
}

inline void Line::set(double theta, double rho)
{
	m_theta = theta;
	m_rho = rho;
}

inline void Line::get(double &theta, double &rho) const
{
	theta = m_theta;
	rho = m_rho;
}

inline void Line::getEquation(double &a, double &b, double &c) const
{
	a = m_a;
	b = m_b;
	c = m_c;
}

} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_LINE_H_ */
