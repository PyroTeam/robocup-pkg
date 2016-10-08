/**
 * \file 		machine.h
 * \class		Machine
 * \brief		classe représentant l'état d'une machine
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef MACHINE_H
#define MACHINE_H

#include "robot.h"

class Machine{

public:

	Machine();
                
	int getX(){return m_x;}
	void setX(int x){m_x=x;}
	int getY(){return m_y;}
	void setY(int y){m_y=y;}
	bool getProcessed(){return m_processed;}
	void setProcessed(bool traite){m_processed=traite;}
	int getRobot(){return m_robot;}


/**
 *	\brief		fait correspondre la machine à une zone
 */	                
	void correspondanceZone();
	
/**
 *	\brief		met à jour les infos concernant la machine
 */
	void updateMachine(Robot tab_robot[]);

private:

	float m_x;
	float m_y;
	int m_robot;
	bool m_processed;

};

#endif
