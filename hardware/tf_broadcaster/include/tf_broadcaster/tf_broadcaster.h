#ifndef _TF_BROADCASTER__TF_BROADCASTER_H_
#define _TF_BROADCASTER__TF_BROADCASTER_H_

/*==========  Includes  ==========*/
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"

/*==========  Prototypes  ==========*/
void poseCallback(const nav_msgs::Odometry& odom, bool onlyOdomToBase = false);

#endif // _TF_BROADCASTER__TF_BROADCASTER_H_