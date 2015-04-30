/**
 * \file 		storageChart.h
 * \brief		fonctions concernant les 6 espaces de stockage disponibles
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */

#ifndef STORAGECHART_H
#define STORAGECHART_H

#define SIZE 6 //seulement 6 espaces de stockage

/**
 *	\brief		vérifie si au moins un produit est en stock
 *	\return		true si au moins un produit en stock sinon false
 */
bool productInStock(Storage tabStock[]);

/**
 *	\brief		détermine l'id d'un produit grâce au date de début et de fin de livraison
 *	\return		id recherché
 */
int findId(Storage tabStock[],int debut, int fin);


#endif
