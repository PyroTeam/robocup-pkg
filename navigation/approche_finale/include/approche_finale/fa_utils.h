#ifndef FA_UTILS_H
#define FA_UTILS_H

#include <ros/ros.h>
#include <vector>
#include "Point.h"

float distance2points(Point a, Point b);
float moy(std::vector<float> position_y);
void asservissement_angle(ros::NodeHandle n,float moy_pente);

#endif
