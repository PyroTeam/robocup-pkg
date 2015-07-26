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

void projection(Segment &worst, Segment &best);

void modify(Segment a, Segment &b);

void adjust(std::list<Segment> &segmentsRecorded, std::list<Segment> segmentsSeen);

std::list<Segment> gather(std::list<Segment> sgts);

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments);

#endif