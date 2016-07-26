/**
* \file        ReportingMachineSrvClient.h
* \class       ReportingMachineSrvClient
* \brief       classe client pour le service de reporter des machines
* \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
* \date        2015-10-10
* \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef REPORTINGMACHINESRVCLIENT_H
#define REPORTINGMACHINESRVCLIENT_H

#include <ros/ros.h>
#include <string>

#include <comm_msg/ReportMachine.h>


class ReportingMachineSrvClient
{
public:
    ReportingMachineSrvClient();
    virtual  ~ReportingMachineSrvClient();
    
    bool reporting(std::string r_name, std::string r_type, uint8_t r_zone);
};

#endif
