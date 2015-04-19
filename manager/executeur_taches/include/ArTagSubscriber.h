#ifndef ARTAGSUBSCRIBER_H
#define ARTAGSUBSCRIBER_H

#include <ros/ros.h>
//#include "manager_msg/Landmarks.h" 

class ArTagSubscriber {
public :
  ArTagSubscriber();
  virtual  ~ArTagSubscriber();
  void tesCallback(/*const manager_msg::Landmarks &msg*/);
private :

};
#endif 

