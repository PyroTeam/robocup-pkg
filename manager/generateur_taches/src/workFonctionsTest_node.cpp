/**
 * \file 		workFonctionsTest_node.cpp
 * \brief		Programme de test pour les fonctions de work.cpp
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */


#include <list> 
#include <iostream> 
#include "task.h"
#include "tasksList.h"
#include "order.h"
#include "work.h"
#include "manager_msg/order.h"

using namespace std;

int main() 
{ 
  
	/*** INITIALISATION ***/
	vector<int> black(1,10);
	bool take[3];
	Product action(0,black);

	list< list<Task> > work;
	for(int i=0;i<5;i++){
		work.push_back(creationListTasksAction(7,action,0,0));
	}
	work.push_back(creationListTasksAction(8,action,90,140));   
	cout << "\n\tInitialement la liste de liste de taches a realiser" << endl;
	cout << "est seulement constituee de cinq fois la tache Decapsuler" << endl;
	cout << "et d une fois la task Destocker" << endl;
	getInfoWork(work);
	cout <<""<<endl;
	vector<int> couleurs(2,0);
	Product product(0,couleurs);
	Order order(product,100,150,2,false);
	cout << "\tLa refbox demande de creer deux fois le product 0" << endl;
  
	/*** TEST DE LA FONCTION ALREADY_IN_WORK ***/
	cout << "\tTest de la fonction alreadyInWork" << endl;
	if(alreadyInWork(work, order))
		cout << "les ordres de la refbox ont deja ete pris en compte" << endl;
	else if(!alreadyInWork(work, order))
		cout << "les ordres de la refbox n ont pas encore ete pris en compte" << endl;
  
	/*** TEST DE LA FONCTION ADD_IN_WORK ***/
	int availableCap = 1;
	cout << "\n\tSeul un cap est disponible : availableCap = " << availableCap << endl;
	cout << "\tOn va donc rajouter les nouveaux ordres dans la liste de liste de taches a realiser" <<endl;
	cout << "\tTest de la fonction addInWork" << endl;
	addInWork(work,order,availableCap);
	getInfoWork(work);
	  
	/*** TEST DE LA FONCTION RATIO_CALCULUS ***/
	cout << "\n\tTest de la fonction ratioCalculus" << endl;
	ratioCalculus(work,5,1,take);
	getInfoWork(work);
	  
	/*** TEST DE LA FONCTION MAX_RATIO ***/
	cout <<"\n\tA present on va vérifier que la fonction maxRatio fonctionne" << endl;
	list<list<Task> >::iterator tmp = maxRatio(work);
	cout << "La tache ayant son ratio le plus grand a pour ratio : " << tmp->begin()->getRatio() << endl;
	cout << " et pour intitule : "<<tmp->begin()->getTitle()<<"\n" << endl;
  
	return 0;
} 
