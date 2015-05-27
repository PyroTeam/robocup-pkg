#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "cartographie_utils.h"

Segment::Segment() : m_angle(0.0),m_size(0.0)
{

}
Segment::~Segment()
{

}

double Segment::getAngle()
{
	return m_angle;
}

double Segment::getSize()
{
	return m_size;
}

geometry_msgs::Point Segment::getMin()
{
	return m_min;
}

geometry_msgs::Point Segment::getMax()
{
	return m_max;
}

void Segment::setAngle(double theta)
{
	m_angle = atan2(tan(theta),1);
}

void Segment::setSize(double size)
{
	m_size = size;
}

void Segment::setMin(geometry_msgs::Point a)
{
	m_min = a;
}
void Segment::setMax(geometry_msgs::Point a)
{
	m_max = a;
}

void Segment::setPoints(geometry_msgs::Point a, geometry_msgs::Point b)
{
	m_min = a;
	m_max = b;
}

void Segment::update()
{
	m_angle = tan((m_max.y-m_min.y)/(m_max.x-m_min.x));
	m_size  = dist(m_min,m_max);
}