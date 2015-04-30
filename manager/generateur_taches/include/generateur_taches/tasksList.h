/**
 * \file 		tasksList.h
 * \brief		fonctions utiles concernant la production d'un produit
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */

#ifndef TASKSLIST_H
#define TASKSLIST_H

#include <list>
#include "task.h"
#include "product.h"

/**
 *	\brief		crée une liste de tâches à partir d'un produit passé en paramètre
 *	\return		une liste de tâches pour la création d'un produit
 */
std::list<Task> creationListTasksProduct(Product product,int beginningDelivery,int endDelivery);

/**
 *	\brief		crée une liste de tâches à partir d'une action et d'un produit passés en paramètre
 *	\return		une liste de tâches pour la création d'une tâche unique
 *				telle que UNCAP DELIVER
 */
std::list<Task> creationListTasksAction(int action,Product product,int beginningDelivery,int endDelivery);

/**
 *	\brief		vérifie si la première tâche de la liste est UNCAP
 *	\return		true si c'est le cas sinon false
 */
bool uncapInWork(std::list<Task> list);

#endif
