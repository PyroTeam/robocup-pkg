#include "Line.h"
#include <cmath>
#include <vector>

Line::Line()
{

}

Line::~Line()
{

}

geometry_msgs::Pose2D Line::getPoint()
{
	geometry_msgs::Pose2D p;
	p.x     = m_point.x;
	p.y     = m_point.y;
	p.theta = m_angle;

	return p;
}

double Line::getAngle()
{
	return m_point.theta;
}

double Line::getPente()
{
	return m_slope;
}

double Line::getYIntercept()
{
	return m_YIntercept;
}

void Line::set(geometry_msgs::Pose2D p, double slope, double ord)
{
	m_point 	 = p;
	m_pente 	 = slope;
	m_YIntercept = ord;
}

void Line::setAngle(double theta)
{
	m_angle = theta;
}

void Line::build(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b)
{
	double slope = (b.y - a.y)/(b.x - a.x);
	double yIntercept = (b.y - a.y) - slope * (b.x - a.x);

  	set(a, slope, yIntercept);
}