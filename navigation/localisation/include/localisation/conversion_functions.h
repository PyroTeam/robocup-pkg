/**
 * \file 			conversion_functions.h
 * \brief			fonctions de conversions nécessaires à la détection de machines et cartographie
 * \author		Danel Thomas
 * \date			2016-07-18
 * \copyright 2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef CONVERSION_FUNCTIONS_H
#define CONVERSION_FUNCTIONS_H

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "deplacement_msg/Machines.h"
#include "deplacement_msg/Landmarks.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "Segment.h"
#include "Machine.h"



geometry_msgs::Point globalToLocal(const geometry_msgs::Point &p, const Segment &s);

std::vector<deplacement_msg::Machine> convertIntoMsg(std::vector<Machine> mps);
std::vector<geometry_msgs::Pose2D> convert(std::vector<Machine> mps);

std::list<Segment> landmarksToSegments(const deplacement_msg::Landmarks &tabSegments);

std::vector<geometry_msgs::Pose2D> backToLandmarks(const std::list<Segment> &vect);

geometry_msgs::Pose2D pointToPose2D(const geometry_msgs::Point &point);
geometry_msgs::Pose2D pointToPose2D(const geometry_msgs::Point &point, double angle);

geometry_msgs::Point pose2DToPoint(const geometry_msgs::Pose2D &pose2d);

#endif
