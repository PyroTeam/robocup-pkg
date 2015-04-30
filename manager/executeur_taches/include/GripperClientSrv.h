#ifndef GRIPPERCLIENTSRV_H
#define GRIPPERCLIENTSRV_H


#include "ros/ros.h"
#include "manager_msg/SetGripper.h"
#include <cstdlib>

class GripperClientSrv {
public :
  GripperClientSrv();
  virtual  ~GripperClientSrv();
 bool gripper_uppdate(bool new_state);
};
#endif