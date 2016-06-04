/**
 * \file        BaseStation.h
 * \class       BaseStation
 * \brief       classe qui stocke les données de la basestation
 * \author      Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date        2015-10-10
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef BASESTATION_H
#define BASESTATION_H

#include "Machine.h"

using namespace manager_msg;

class BaseStation : public Machine
{
  	private:
	
		/* Variables d'instance*/
		int m_redBase;
		int m_blackBase;
		int m_silverBase;
	  
  	public:
	
		/* Constructeur */
		BaseStation();

		/* Déstructeur */
		virtual ~BaseStation();

		virtual void FonctionVirtuelle();

		/* Méthodes */   
		int getRedBase();
		int getBlackBase();      
		int getSilverBase();
		void majRed(int nbRouge);
		void majBlack(int nbNoir);
		void majSilver(int nbArgent);
		void take_base(int color,int n_robot,int n_order);
		void bring_base_rs(int color,int n_robot,int n_order,int machine);
};

#endif
