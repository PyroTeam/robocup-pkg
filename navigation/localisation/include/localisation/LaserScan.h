#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <vector>
#include <list>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Point.h"

#include "Line.h"
#include "Model.h"
#include "Segment.h"

class laserScan
{
public:
	laserScan();
	~laserScan();	

	void PolarToCart();

	float getRangeMin();
	float getRangeMax();
	float getAngleMin();
	float getAngleMax();
	float getAngleInc();

	const std::vector<float>& getRanges() const;
	const std::list<geometry_msgs::Point>& getPoints() const;

	void setRangeMin(float min);
	void setRangeMax(float max);
	void setAngleMin(float min);
	void setAngleMax(float max);
	void setAngleInc(float inc);

	void set(const sensor_msgs::LaserScanConstPtr& scan);

	void laserCallback(const sensor_msgs::LaserScanConstPtr& scan); 

private:
	std::vector<float> m_ranges;
	std::list<geometry_msgs::Point>   m_points;

	float m_range_min;
	float m_range_max;
	float m_angle_min;
	float m_angle_max;
	float m_angle_inc;
public:
	ros::Time m_stamp;
};

#endif