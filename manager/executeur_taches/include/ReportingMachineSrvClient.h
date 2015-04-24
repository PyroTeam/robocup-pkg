#ifndef REPORTINGMACHINESRVCLIENT_H
#define REPORTINGMACHINESRVCLIENT_H

#include <ros/ros.h>
#include <string>

#include <comm_msg/ReportMachine.h>


class ReportingMachineSrvClient{
public :
	ReportingMachineSrvClient();
	virtual  ~ReportingMachineSrvClient();
	bool reporting(std::string r_name, std::string r_type, uint8_t r_zone);
};
#endif