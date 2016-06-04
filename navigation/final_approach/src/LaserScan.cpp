#include "final_approach/LaserScan.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

LaserScan::LaserScan():m_rangeMin(0.0),m_rangeMax(0.0),m_angleMin(0.0),m_angleMax(0.0),m_angleInc(0.0)
{
	m_lsSub = m_nh.subscribe("hardware/scan", 1, &LaserScan::laserCallback, this);
}

void LaserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	setRangeMin(scan->range_min);
	setRangeMax(scan->range_max);
	setAngleMin(scan->angle_min);
	setAngleMax(scan->angle_max);
	setAngleInc(scan->angle_increment);
	m_ranges = scan->ranges;
	m_stamp = scan->header.stamp;
	m_frame = scan->header.frame_id;
}


LaserScan::~LaserScan(){}

