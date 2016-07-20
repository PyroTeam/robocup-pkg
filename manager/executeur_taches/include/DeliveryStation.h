/**
 * \file        DeliveryStation.h
 * \class       DeliveryStation
 * \brief       classe qui stocke les données de la deliverystation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef DELIVERY_STATION_H
#define DELIVERY_STATION_H

#include "Machine.h"

using namespace manager_msg;

class DeliveryStation : public Machine
{
public:
		/* Constructeur */
		DeliveryStation(int teamColor);

		/* Destructeur */
		virtual ~DeliveryStation();

		/* Méthodes */
		void deliver();
};

#endif
