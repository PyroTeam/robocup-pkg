#ifndef ROBOT_H
#define ROBOT_H

#include "manager_msg/activity.h"

class Robot{
 
public :
 
	Robot(){m_occuped=true; m_machine=manager_msg::activity::NONE; m_nbOrder=0;}
 
	bool getOccuped(){return m_occuped;}
	void setOccuped(bool occuped){m_occuped = occuped;}
	int getMachine(){return m_machine;}
	void setMachine(int machine){m_machine=machine;}
	int getNbOrder(){return m_nbOrder;}
	void setNbOrder(int ordre){m_nbOrder=ordre;}
  
private :
  
	bool m_occuped; //indique si le robot est occupe ou non
	int m_machine;
	int m_nbOrder;

};

#endif
