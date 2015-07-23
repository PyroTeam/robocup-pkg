#ifndef MATH_FUNCTION_H
#define MATH_FUNCTION_H

#include "geometry_msgs/Point.h"
#include "Segment.h"
#include "Line.h"

double dist(geometry_msgs::Point a, Line d);

double dist(geometry_msgs::Point a, Segment s);

double dist(geometry_msgs::Point a, geometry_msgs::Point b);

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m);

geometry_msgs::Point middle(Segment s);

double dist(Segment seg1, Segment seg2);

double angle(Segment a, Segment b);

geometry_msgs::Point ortho(geometry_msgs::Point a, Line d);

geometry_msgs::Point ortho(geometry_msgs::Point a, Segment s);

double linReg(const std::list<geometry_msgs::Point> &points, geometry_msgs::Pose2D &p);

#endif