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

	float getRangeMin() const;
	float getRangeMax() const;
	float getAngleMin() const;
	float getAngleMax() const;
	float getAngleInc() const;

	const std::vector<float>& getRanges() const;
	const std::list<geometry_msgs::Point>& getPoints() const;

	void setRangeMin(const float &min);
	void setRangeMax(const float &max);
	void setAngleMin(const float &min);
	void setAngleMax(const float &max);
	void setAngleInc(const float &inc);

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