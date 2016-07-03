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

const ros::Time& laserScan::getTime() const
{
	return m_stamp;
}

const std::vector<float>& laserScan::getRanges() const
{
	return m_ranges;
}

const std::list<geometry_msgs::Point>& laserScan::getPoints() const
{
	return m_points;
}

const std::vector<geometry_msgs::Point>& laserScan::getPointVector() const
{
	return m_points_vector;
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
		// sur le simulateur, le bruit sur le laser est important entre 5.5 et 5.6 m,
		// on réduit alors le seuil de validité du scan pour ne pas avoir des rayons
		// qui ne toucheraient pas d'obstacles mais serainet considérés comme tels
	   	if((m_ranges[i] > getRangeMin()) && (m_ranges[i] < getRangeMax()-0.5))
	   	{
	   		geometry_msgs::Point p;
	   		p.x = m_ranges[i]*cos(double(getAngleMin() + i*getAngleInc()));
	   		p.y = m_ranges[i]*sin(double(getAngleMin() + i*getAngleInc()));
			  m_points.push_back(p);
        m_points_vector.push_back(p);
		}
	}
}

void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	m_points.clear();

	m_range_min = scan->range_min;
	m_range_max = scan->range_max;
	m_angle_min = scan->angle_min;
	m_angle_max = scan->angle_max;
	m_angle_inc = scan->angle_increment;
	m_ranges = scan->ranges;
	m_stamp = scan->header.stamp;

	this->PolarToCart();
  m_received_data = true;
}
