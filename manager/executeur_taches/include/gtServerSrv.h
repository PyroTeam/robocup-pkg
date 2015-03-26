#ifndef GTSERVERSRV_H
#define GTSERVERSRV_H

#include "ros/ros.h"
#include "manager_msg/order.h"

class gtServerSrv {
public :
	gtServerSrv();
	virtual  ~gtServerSrv();
	bool responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res);
	void setId(int id);
private :
	int nb_robot;
	int m_id;
};

#endif