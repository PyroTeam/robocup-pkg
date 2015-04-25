#ifndef FAKECLIENTSRV_H
#define FAKECLIENTSRV_H


#include "ros/ros.h"
#include "manager_msg/SetGripper.h"
#include <cstdlib>

class fakeClientSrv {
public :
  fakeClientSrv();
  virtual  ~fakeClientSrv();
private :
 bool gripper_uppdate();
};
#endif