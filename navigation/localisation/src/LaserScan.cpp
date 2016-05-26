#include "LaserScan.h"
#include <cmath>

laserScan::laserScan():m_range_min(0.0),m_range_max(0.0),m_angle_min(0.0),m_angle_max(0.0),m_angle_inc(0.0)
{

}

laserScan::~laserScan()
{

}

float laserScan::getRangeMin() const
{
	return m_range_min;
}

float laserScan::getRangeMax() const
{
	return m_range_max;
}

float laserScan::getAngleMin() const
{
	return m_angle_min;
}

float laserScan::getAngleMax() const
{
	return m_angle_max;
}

float laserScan::getAngleInc() const
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

void laserScan::setRangeMin(const float &min)
{
	m_range_min = min;
}
void laserScan::setRangeMax(const float &max)
{
	m_range_max = max;
}

void laserScan::setAngleMin(const float &min)
{
	m_angle_min = min;
}

void laserScan::setAngleMax(const float &max)
{
	m_angle_max = max;
}

void laserScan::setAngleInc(const float &inc)
{
	m_angle_inc = inc;
}


void laserScan::PolarToCart ()
{
	for (int i=0; i<m_ranges.size(); i++)
	{
	   	if((m_ranges[i] > getRangeMin()) && (m_ranges[i] < getRangeMax()-0.5))
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

	this->setRangeMin(scan->range_min);
	this->setRangeMax(scan->range_max);
	this->setAngleMin(scan->angle_min);
	this->setAngleMax(scan->angle_max);
	this->setAngleInc(scan->angle_increment);

	m_ranges=scan->ranges;

	this->PolarToCart();
}

void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	set(scan);
	m_stamp = scan->header.stamp;
}
