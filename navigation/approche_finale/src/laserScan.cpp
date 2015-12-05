#include "laserScan.h"
#include <ros/ros.h>
#include <cmath>
#include "sensor_msgs/LaserScan.h"

laserScan::laserScan():m_rangeMin(0.0),m_rangeMax(0.0),m_angleMin(0.0),m_angleMax(0.0),m_angleInc(0.0)
{
	m_lsSub = m_nh.subscribe("hardware/scan",1000,&laserScan::laserCallback,this);
}

void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	setRangeMin(scan->range_min);
	setRangeMax(scan->range_max);
	setAngleMin(scan->angle_min);
	setAngleMax(scan->angle_max);
	setAngleInc(scan->angle_increment);
	m_ranges = scan->ranges;

}


laserScan::~laserScan(){}

