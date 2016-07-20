/**
 * \file        BaseStation.h
 * \class       BaseStation
 * \brief       classe qui stocke les données de la basestation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef BASE_STATION_H
#define BASE_STATION_H

#include "Machine.h"

using namespace manager_msg;

/*
  Sur la BS, c'est le rôle du replenisher de s'assurer qu'il y a toujours
  assez de bases de chaque couleur, la seule action sur cette machine est
  donc prendre une base de la couleur demandée

  Pas besoin de compter les bases restantes de chaque couleur
  Pas besoin d'une fonction allant apporter une base à une RS, c'est typiquement
  le rôle de l'executeur
*/

class BaseStation : public Machine
{
public:

		/* Constructeur */
		BaseStation(int teamColor);

		/* Destructeur */
		virtual ~BaseStation();

		/* Méthodes */
		bool take(int color);
};

#endif
