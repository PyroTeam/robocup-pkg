/**
 * \file 		storageChart.h
 * \brief		fonctions concernant les 6 espaces de stockage disponibles
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef TABMACHINE_H
#define TABMACHINE_H

#define NBR_MACHINES 6

#include "machine.h"
#include "robot.h"


/**
 *	\brief		vérifie si toutes les machines ont été explorées
 *	\return		true si toutes les machines explorées sinon false
 */
bool finishedExploration(Machine tabMachine[]);

/**
 *	\brief		met à jour les zones explorées ou non
 */
void updateZone(Machine (&tabMachine)[NBR_MACHINES], Robot tab_robot[]);

#endif
