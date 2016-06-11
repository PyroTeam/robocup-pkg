#ifndef MATH_FUNCTION_H
#define MATH_FUNCTION_H

#include "geometry_msgs/Point.h"
#include "Segment.h"
#include "Line.h"

double dist(const geometry_msgs::Point &a, const Line &d);

double dist(const geometry_msgs::Point &a, const Segment &s);

double dist(const geometry_msgs::Point &a, const geometry_msgs::Point &b);

double dist(const geometry_msgs::Pose2D &c, const geometry_msgs::Pose2D &m);

geometry_msgs::Point middle(const Segment &s);

double dist(const Segment &seg1, const Segment &seg2);

double angle(const Segment &a, const Segment &b);

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const Line &d);

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const Segment &s);

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const geometry_msgs::Pose2D &p);

double linReg(const std::list<geometry_msgs::Point> &points, geometry_msgs::Pose2D &p);

#endif