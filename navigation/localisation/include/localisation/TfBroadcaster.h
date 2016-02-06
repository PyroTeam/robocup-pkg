#ifndef _TF_BROADCASTER_H_
#define _TF_BROADCASTER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"

void poseCallback(const nav_msgs::Odometry& odom);

#endif // _TF_BROADCASTER_H_