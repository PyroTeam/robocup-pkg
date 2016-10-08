/**
 * \file        MPS.h
 * \class       MPS
 * \brief       classe pour stocker la position et l'orientation d'une machine
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef MPS_H
#define MPS_H
class MPS
{
	public:
		/* Constructeur */
		MPS();

		/* Déstructeur */
		~MPS();

		/* Variables d'instance */
		float  x;
		float  y;
		float  theta;
		int    zone;
		bool   isHere;
	
};

#endif