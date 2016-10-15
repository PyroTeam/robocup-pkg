/**
 * \file 			math_functions.h
 * \brief			fonctions mathématiques spécifiques à la détection de segments
 * \author		Danel Thomas
 * \date			2016-07-18
 * \copyright 2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef MATH_FUNCTION_H
#define MATH_FUNCTION_H

#include "geometry_msgs/Point.h"
#include "Segment.h"
#include "Line.h"

#include "geometry_utils/geometry_utils.h"

double dist(const geometry_msgs::Point &a, const Line &d);
double dist(const geometry_msgs::Point &a, const Segment &s);


geometry_msgs::Point middle(const Segment &s);

double dist(const Segment &seg1, const Segment &seg2);

double angle(const Segment &a, const Segment &b);

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const Line &d);

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const Segment &s);

geometry_msgs::Point ortho(const geometry_msgs::Point &a, const geometry_msgs::Pose2D &p);

#endif
