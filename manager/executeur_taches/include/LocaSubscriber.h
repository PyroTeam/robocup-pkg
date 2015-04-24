#ifndef LOCASUBSCRIBER_H
#define LOCASUBSCRIBER_H

#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "manager_msg/Landmarks.h" 
#include "MPS.h"
#include <vector>

class LocaSubscriber {
public :
  LocaSubscriber();
  virtual  ~LocaSubscriber();
  void tesCallback(const manager_msg::LandmarksConstPtr &msg);
  int getZone(float x, float y);
  std::vector<geometry_msgs::Pose2D> tab_machine;
  std::vector<MPS> machine;
};
#endif 

