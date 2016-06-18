/**
 * \file        ReportingMachineSrvClient.h
 * \class       ReportingMachineSrvClient
 * \brief       classe client pour le service de reporter des machines
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef REPORTINGMACHINESRVCLIENT_H
#define REPORTINGMACHINESRVCLIENT_H

#include <ros/ros.h>
#include <string>

#include <comm_msg/ReportMachine.h>


class ReportingMachineSrvClient
{
	public:
		/* Constructeur */
		ReportingMachineSrvClient();

		/* Destructeur */
		virtual  ~ReportingMachineSrvClient();
		
		/* Méthodes */
		bool reporting(std::string r_name, std::string r_type, uint8_t r_zone);
};

#endif