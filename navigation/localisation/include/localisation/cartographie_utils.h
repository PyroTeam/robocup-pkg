#ifndef CARTOGRAPHIE_H
#define CARTOGRAPHIE_H

#include <ros/ros.h>
#include <cmath>
#include "geometry_msgs/Pose2D.h"

int getArea(geometry_msgs::Pose2D m);

geometry_msgs::Pose2D getCenter(int zone);

#endif
