#ifndef SEGMENT_H
#define SEGMENT_H

#include "geometry_msgs/Point.h"

class Segment{
public:
	Segment();
	~Segment();

	double getAngle();
	double getSize();

	geometry_msgs::Point getMin();
	geometry_msgs::Point getMax();
	geometry_msgs::Point getCenter();

	void setAngle(double theta);
	void setSize(double size);
	void setMin(geometry_msgs::Point a);
	void setMax(geometry_msgs::Point a);
	void setPoints(geometry_msgs::Point a, geometry_msgs::Point b);
	
	void update();
	void build(const std::list<geometry_msgs::Point> &points);
protected:
	double			 		m_angle;
	double 			 		m_size;
	geometry_msgs::Point 	m_min;
	geometry_msgs::Point 	m_max;
};

#endif