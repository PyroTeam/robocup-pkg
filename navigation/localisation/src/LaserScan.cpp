#include "laserScan.h"
#include <cmath>

laserScan::laserScan():m_range_min(0.0),m_range_max(0.0),m_angle_min(0.0),m_angle_max(0.0),m_angle_inc(0.0)
{

}

laserScan::~laserScan()
{

}

float laserScan::getRangeMin()
{
	return m_range_min;
}

float laserScan::getRangeMax()
{
	return m_range_max;
}

float laserScan::getAngleMin()
{
	return m_angle_min;
}

float laserScan::getAngleMax()
{
	return m_angle_max;
}

float laserScan::getAngleInc()
{
	return m_angle_inc;
}

const std::vector<float>& laserScan::getRanges() const
{
	return m_ranges;
}

const std::list<geometry_msgs::Point>& laserScan::getPoints() const
{
	return m_points;
}

void laserScan::setRangeMin(float min)
{
	m_range_min=min;
}	
void laserScan::setRangeMax(float max)
{
	m_range_max=max;
}

void laserScan::setAngleMin(float min)
{
	m_angle_min=min;
}

void laserScan::setAngleMax(float max)
{
	m_angle_max=max;
}
	
void laserScan::setAngleInc(float inc)
{
	m_angle_inc=inc;
}


void laserScan::PolarToCart ()
{
	for (int i=0; i<m_ranges.size(); i++)
	{
	   	if((m_ranges[i] > getRangeMin()) && (m_ranges[i] < getRangeMax()))
	   	{
	   		geometry_msgs::Point p;
	   		p.x = m_ranges[i]*cos(double(getAngleMin() + i*getAngleInc()));
	   		p.y = m_ranges[i]*sin(double(getAngleMin() + i*getAngleInc()));
			m_points.push_back(p);
		}
	}
}

void laserScan::set(const sensor_msgs::LaserScanConstPtr& scan)
{
	m_points.clear();
	
	setRangeMin(scan->range_min);
	setRangeMax(scan->range_max);
	setAngleMin(scan->angle_min);
	setAngleMax(scan->angle_max);
	setAngleInc(scan->angle_increment);

	m_ranges=scan->ranges;

	PolarToCart();
}

void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	set(scan);
	m_stamp = scan->header.stamp;
}