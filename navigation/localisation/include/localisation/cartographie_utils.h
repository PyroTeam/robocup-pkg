#ifndef CARTOGRAPHIE_H
#define CARTOGRAPHIE_H

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "deplacement_msg/Landmarks.h"
#include "Machine.h"
#include "Segment.h"
#include "landmarks_detection_utils.h"
#include "conversion_functions.h"

int getZone(geometry_msgs::Pose2D m);

geometry_msgs::Pose2D getCenter(int zone);

int machineToArea(geometry_msgs::Pose2D m);

bool isAlmostTheSame(Segment a, Segment b);

bool checkAndChangeIfItsNecessary(Segment s, std::vector<Segment> &vect);

void maj(std::vector<Segment> &segmentsRecorded, std::vector<Segment> segmentsSeen);

#endif