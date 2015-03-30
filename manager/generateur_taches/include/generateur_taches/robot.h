#ifndef ROBOT_H
#define ROBOT_H

class Robot{
 public :
  Robot(){m_occupe = false;}
  bool get_occupe(){return m_occupe;}
  void set_occupe(bool occupe){m_occupe = occupe;}
 private :
  bool m_occupe; //indique si le robot est occupe ou non
};
#endif
