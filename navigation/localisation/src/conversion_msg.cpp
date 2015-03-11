#include "ros/ros.h"
#include "Modele.h"
#include "Machine.h"
#include "conversion_msg.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Machines.h"

geometry_msgs::Pose2D convertMachineToPose2D(Machine m){
  geometry_msgs::Pose2D pose2d;
  ROS_INFO("Conversion machine to pose2D OK");

  pose2d.x     = m.getX();
  pose2d.y     = m.getY();
  pose2d.theta = m.getOrientation();

  return pose2d;
}

deplacement_msg::Machines fillTabFrom(std::list<Machine> listOfMachines){
  deplacement_msg::Machines tab;

  int i = 0;
  for (auto &it : listOfMachines){ 
    geometry_msgs::Pose2D msgMachine;
    msgMachine = convertMachineToPose2D(it);
      
    tab.machines.push_back(msgMachine);

    i++;
  }
  return tab;
}