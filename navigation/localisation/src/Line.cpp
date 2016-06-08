#include "Line.h"
#include <cmath>
#include <vector>

Line::Line()
{

}

Line::~Line()
{

}

geometry_msgs::Pose2D Line::getPoint() const
{
	return m_point;
}

double Line::getAngle() const
{
	return m_point.theta;
}

double Line::getSlope() const
{
	return tan(m_point.theta);
}

void Line::set(geometry_msgs::Pose2D p)
{
	m_point = p;
}

void Line::build(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b)
{
	geometry_msgs::Pose2D p;
	p.x = (b.x + a.x)/2;
	p.y = (b.y + a.y)/2;
	p.theta = (b.y - a.y)/(b.x - a.x);

  set(p);
}
