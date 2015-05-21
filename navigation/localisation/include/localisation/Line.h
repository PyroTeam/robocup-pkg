#ifndef LINE_H
#define LINE_H

#include "geometry_msgs/Pose2D.h"

class Line{
public:
	Line();
	~Line();

	geometry_msgs::Pose2D getPoint();
	double getAngle();
	double getSlope();

	void set(geometry_msgs::Pose2D p);
	void build(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b);

private:
	geometry_msgs::Pose2D  	m_point;
};

#endif