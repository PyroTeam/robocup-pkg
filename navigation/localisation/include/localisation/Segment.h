/**
 * \file 			Segment.h
 * \brief			classe Segment
 * \author		Danel Thomas
 * \date			2016-07-18
 * \copyright 2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef SEGMENT_H
#define SEGMENT_H

#include "geometry_msgs/Point.h"

class Segment{
public:
	Segment();
	~Segment();

	double getAngle() const;
	double getSize() const;

	geometry_msgs::Point getMin() const;
	geometry_msgs::Point getMax() const;
	geometry_msgs::Point getCenter() const;

	void setAngle(const double &theta);
	void setSize(const double &size);
	void setMin(const geometry_msgs::Point &a);
	void setMax(const geometry_msgs::Point &a);
	void setPoints(const geometry_msgs::Point &a, const geometry_msgs::Point &b);

	void update();
	void build(const std::list<geometry_msgs::Point> &points);

protected:
	double			 		m_angle;
	double 			 		m_size;
	geometry_msgs::Point 	m_min;
	geometry_msgs::Point 	m_max;
};

#endif
