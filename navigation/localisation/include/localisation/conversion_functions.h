#ifndef CONVERSION_FUNCTIONS_H
#define CONVERSION_FUNCTIONS_H

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "deplacement_msg/Landmarks.h"
#include "Segment.h"
#include "Machine.h"

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p, geometry_msgs::Pose2D odomRobot);

geometry_msgs::Point globalToLocal(geometry_msgs::Point p, Segment s);

deplacement_msg::Landmarks convert(std::vector<Machine> mps);

std::list<Segment> landmarksToSegments(deplacement_msg::Landmarks tabSegments);

deplacement_msg::Landmarks backToLandmarks(std::list<Segment> vect);

geometry_msgs::Pose2D pointToPose2D(geometry_msgs::Point point);

geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D pose2d);

#endif