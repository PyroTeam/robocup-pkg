#ifndef ARTAGCLIENTSRV_H
#define ARTAGCLIENTSRV_H

#include <ros/ros.h>
#include <manager_msg/artag.h>

class ArTagClienSrv {
public :
  ArTagClienSrv();
  virtual  ~ArTagClienSrv();
  int16_t askForId();
private :
	int16_t m_id;	
};
#endif 



