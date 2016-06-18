/**
 * \file        DeliveryStation.h
 * \class       DeliveryStation
 * \brief       classe qui stocke les données de la deliverystation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef DELIVERYSTATION_H
#define DELIVERYSTATION_H

#include "Machine.h"

using namespace manager_msg;

class DeliveryStation : public Machine
{
	private:

	public:

		/* Constructeur */
		DeliveryStation(int teamColor);

		/* Déstructeur */  
		virtual ~DeliveryStation();

		/* Méthodes */
		virtual void FonctionVirtuelle();

		void deliverToDS(int n_robot, int n_order); 
  
};

#endif