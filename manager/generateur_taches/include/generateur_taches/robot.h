#ifndef ROBOT_H
#define ROBOT_H

class Robot{
 
 public :
 
  Robot(){m_occupe = false;}
  
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
