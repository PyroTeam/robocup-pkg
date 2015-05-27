#ifndef CARTOGRAPHIE_H
#define CARTOGRAPHIE_H

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "deplacement_msg/Landmarks.h"
#include "Machine.h"
#include "Segment.h"
#include "landmarks_detection_utils.h"

double dist(geometry_msgs::Point a, geometry_msgs::Point b);

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m);

geometry_msgs::Point middle(Segment s);

double dist(Segment seg1, Segment seg2);

double angle(Segment a, Segment b);

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p, geometry_msgs::Pose2D odomRobot);

geometry_msgs::Point globalToLocal(geometry_msgs::Point p, Segment s);

int getZone(geometry_msgs::Pose2D m);

geometry_msgs::Pose2D getCenter(int zone);

int machineToArea(geometry_msgs::Pose2D m);

deplacement_msg::Landmarks convert(std::vector<Machine> mps);

std::vector<Segment> landmarksToSegments(deplacement_msg::Landmarks tabSegments);

bool isAlmostTheSame(Segment a, Segment b);

bool checkAndChangeIfItsNecessary(Segment s, std::vector<Segment> &vect);

void maj(std::vector<Segment> &segmentsRecorded, std::vector<Segment> segmentsSeen);

deplacement_msg::Landmarks backToLandmarks(std::vector<Segment> vect);

#endif