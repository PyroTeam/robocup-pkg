#ifndef EKF_FUNCTIONS_H
#define EKF_FUNCTIONS_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <eigen/dense>
#include <string>

void initPosRobot(string s,int n);

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D PosInitRobot, geometry_msgs::Pose2D odomRobot);

#endif