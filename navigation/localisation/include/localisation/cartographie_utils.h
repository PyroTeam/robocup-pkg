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

bool isTheSame(Segment a, Segment b);

bool isAlmostTheSame(Segment a, Segment b);

void projection(Segment &worst, Segment &best);

bool modify(Segment a, Segment &b);

void adjust(std::list<Segment> &segmentsRecorded, std::list<Segment> segmentsSeen);

void gatherTwoSegments(Segment &segment_1, Segment segment_2);

void gatherOneSegmentWithAList(Segment &segment, std::list<Segment> &sgts);

void gather(std::list<Segment> &sgts);

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments);

#endif