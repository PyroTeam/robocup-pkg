#ifndef EXPLOINFOSUBSCRIBER_H
#define EXPLOINFOSUBSCRIBER_H

#include <ros/ros.h>
#include "manager_msg/ExplorationInfo.h" 
#include "manager_msg/ExplorationSignal.h" 
#include "manager_msg/ExplorationZone.h"
#include "manager_msg/LightSpec.h"
#include "FeuClientAction.h" 
#include <string>

class ExploInfoSubscriber {
public :
  ExploInfoSubscriber();
  virtual  ~ExploInfoSubscriber();
  void tesCallback(const manager_msg::ExplorationInfo &msg);
  void interpretationFeu();
  std::string type;
  uint8_t zone;
private :
  std::vector<manager_msg::ExplorationSignal>  m_signals;
  std::vector<manager_msg::ExplorationZone>  m_zones;
};
#endif 

