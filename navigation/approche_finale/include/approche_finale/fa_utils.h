#ifndef FA_UTILS_H
#define FA_UTILS_H

#include <ros/ros.h>
#include <vector>
#include "Point.h"

float distance2points(Point a, Point b);
float moy(std::vector<float> position_y);
int asservissement_angle(ros::Publisher pub_mvt,float moy_pente);
int asservissement_position_y(ros::Publisher pub_mvt, float moy_pos,float objectif, float ortho);
int asservissement_position_x(ros::Publisher pub_mvt, float distance);

#endif
