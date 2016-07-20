/**
 * \file        MyElements.h
 * \class       MyElements
 * \brief       classe pour les différentes machines
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef MYELEMENTS_H
#define MYELEMENTS_H

#include "common_utils/types.h"
#include "manager_msg/order.h"

#include "Machine.h"
#include "BaseStation.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"

class MyElements
{
	private:

		/* Variables d'instance*/
		BaseStation m_bs;
		RingStation m_rs1;
		RingStation m_rs2;
		CapStation m_cs1;
		CapStation m_cs2;
		DeliveryStation m_ds;

    int m_color;

	public:

		/* Constructeur */
		MyElements(int teamColor);

		/* Destructeur */
		virtual ~MyElements();

		/* Méthodes */
		BaseStation& getBS();
		RingStation& getRS1();
		RingStation& getRS2();
		RingStation& getRS(int color);
		CapStation&  getCS1();
		CapStation&  getCS2();
		CapStation&  getCS(int color);
		DeliveryStation& getDS();
		Machine *getMachineFromTag(int arTag);

};

#endif
