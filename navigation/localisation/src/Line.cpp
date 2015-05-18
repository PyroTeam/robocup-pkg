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
	return m_point;
}

double Line::getAngle()
{
	return m_point.theta;
}

double Line::getSlope()
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
	m_slope 	 = slope;
	m_YIntercept = ord;
}

void Line::build(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b)
{
	double dX = b.x - a.x;
	double dY = b.y - a.y;
	double slope = dY/dX;
	double yIntercept = dY - slope*dX;

  	set(a, slope, yIntercept);
}