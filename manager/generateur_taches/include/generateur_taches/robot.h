#ifndef ROBOT_H
#define ROBOT_H

#include "manager_msg/activity.h"

class Robot{
 
 public :
 
  Robot(){m_occupe=true; m_machine=manager_msg::activity::NONE; m_nb_ordre=0;}
  
  bool get_occupe(){return m_occupe;}
  void set_occupe(bool occupe){m_occupe = occupe;}
  int get_machine(){return m_machine;}
  void set_machine(int machine){m_machine=machine;}
  int get_nb_ordre(){return m_nb_ordre;}
  void set_nb_ordre(int ordre){m_nb_ordre=ordre;}
  
 private :
  
  bool m_occupe; //indique si le robot est occupe ou non
  int m_machine;
  int m_nb_ordre;
};
#endif
