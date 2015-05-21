#ifndef SEGMENT_H
#define SEGMENT_H

#include "geometry_msgs/Point.h"
#include <list>

#include <vector>
#include <list>
#include <cmath>

class Segment{
public:
	Segment();
	~Segment();

	double getAngle();
	double getSize();

	geometry_msgs::Point getMin();
	geometry_msgs::Point getMax();

	void setAngle(double theta);
	void setSize(double size);
	void setPoints(geometry_msgs::Point a, geometry_msgs::Point b);
	
protected:
	double			 		m_angle;
	double 			 		m_size;
	geometry_msgs::Point 	m_min;
	geometry_msgs::Point 	m_max;
};

#endif