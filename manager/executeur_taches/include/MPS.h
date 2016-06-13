/**
 * \file        MPS.h
 * \class       MPS
 * \brief       classe pour stocker la position et l'orientation d'une machine
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   PyroTeam, Polytech-Lille
 */

#ifndef MPS_H
#define MPS_H

#include "geometry_msgs/Pose2D.h"


class MPS
{
	public:
		/* Constructeur */
		MPS();

		/* Destructeur */
		~MPS();

		/* Variables d'instance */
		geometry_msgs::Pose2D pose;
		int    zone;
		bool   isHere;

};

#endif
