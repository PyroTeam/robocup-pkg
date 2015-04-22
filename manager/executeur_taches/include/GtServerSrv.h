#ifndef GTSERVERSRV_H
#define GTSERVERSRV_H

#include "ros/ros.h"
#include "manager_msg/order.h"
#include "manager_msg/activity.h"
#include "manager_msg/finalApproachingAction.h"

#include "ExploInfoSubscriber.h"
#include "Machine.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"
#include "BaseStation.h"
#include "MyElements.h"

class GtServerSrv {
public :
	GtServerSrv();
	virtual  ~GtServerSrv();
	bool responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res);
	void setId(int id);
	manager_msg::activity getActivityMsg();
	manager_msg::finalApproachingAction getFinalAppAction();
private :
	int nb_robot;
	int m_id;
	manager_msg::activity m_msg;
	manager_msg::finalApproachingAction m_act;
	ExploInfoSubscriber *m_ei;
};

#endif