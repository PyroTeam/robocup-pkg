/**
 * \file        CapStation.h
 * \class       CapStation
 * \brief       classe qui stocke les données de la capstation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef CAP_STATION_H
#define CAP_STATION_H

#include "Machine.h"

using namespace manager_msg;

class CapStation : public Machine
{
  	private:

		/* Variables d'instance*/
		int m_blackCap;
		int m_greyCap;
		int m_stockID[3];
		int m_capID[3];

  	public:

		/* Constructeur */
		CapStation(int teamColor);
		CapStation(int teamColor, int nb);

		/* Destructeur */
		virtual ~CapStation();

		/* Méthodes */
		int getGreyCap();
		int getBlackCap();
		int getStockage(int i);

		void majStockID(int i, int val);
		void majBlack(int nbNoir);
		void majGrey(int nbGris);

		void put_cap(int color);
		void take_cap();

		void uncap();
		void stock(int id);
		void destock(int id);
};

#endif
