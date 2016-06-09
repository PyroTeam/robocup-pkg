#ifndef CARTOGRAPHIE_H
#define CARTOGRAPHIE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include "Machine.h"
#include "Segment.h"
#include "Ekf.h"

#include "landmarks_detection_utils.h"
#include "conversion_functions.h"
#include "math_functions.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"

#include <Eigen/Dense>
#include <vector>
#include <cmath>

int getArea(geometry_msgs::Pose2D m);

geometry_msgs::Pose2D getCenter(int zone);

int machineToArea(geometry_msgs::Pose2D m);

bool isTheSame(Segment a, Segment b);

bool isAlmostTheSame(Segment a, Segment b);

void projection(Segment &worst, Segment &best);

bool improve(Segment a, Segment &b);

void adjust(std::list<Segment> &segmentsRecorded, std::list<Segment> segmentsSeen);

void gatherTwoSegments(Segment &segment_1, Segment segment_2);

void gatherOneSegmentWithAList(Segment &segment, std::list<Segment> &sgts);

void gather(std::list<Segment> &sgts);


#endif
