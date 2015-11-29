/**
 * \file 		ArTagClientSrv.h
 * \class		ArTagClientSrv
 * \brief		classe client service ArTag
 * \author		Hage Chehade Sandra (sandra.hage-chehade@polytech-lille.net)
 * \date		2015-10-10
 * \copyright	PyroTeam, Polytech-Lille
 */

#ifndef ARTAGCLIENTSRV_H
#define ARTAGCLIENTSRV_H

#include <ros/ros.h>
#include <trait_im_msg/artag.h>

class ArTagClienSrv 
{
	public:
		/* Constructeur */
		ArTagClienSrv();

		/* Déstructeur */
		virtual  ~ArTagClienSrv();

		/* Variables d'instance */
		int16_t askForId();
	private:
		int16_t m_id;	
};
#endif 



