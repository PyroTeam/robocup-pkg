#include <list>
#include <iostream>
#include "travail.h"
#include "tache.h"
#include "listetaches.h"
#include "ordre.h"

#include <iostream>

using namespace std;

// verifie si la demande de la RefBox a déjà prise en compte
bool deja_dans_travail(list<list<Tache> > travail, Ordre order){
  bool tmp = false;
  list<list<Tache> >::iterator travail_iterator;
  for(travail_iterator = travail.begin(); travail_iterator != travail.end(); travail_iterator++){
    if( (order.get_produit().get_nbr_ring() == travail_iterator->begin()->get_produit()) 
	&& (order.get_debut_livraison() == travail_iterator->begin()->get_debut_livraison()) ){
      tmp =true;
    }
  }
  return tmp;
}

// rajoute les nouvelles listes de tâches à faire éxecuter 
void rajout_dans_travail(list<list<Tache> > &travail, Ordre &order,int &cap_dispo){
  if(!deja_dans_travail(travail,order) && !order.get_traite()){
    order.set_traite(true);
    for(int i=0; i< order.get_quantite(); i++){
      if(cap_dispo > 0){
	travail.push_back(creation_liste_taches_prod(order.get_produit(),order.get_debut_livraison(),order.get_fin_livraison()));
	cap_dispo --;
      }
      else{
	list<list<Tache> >::iterator travail_iterator;
	travail_iterator = travail.begin();
	while(travail_iterator != travail.end() && !decapsuler_dans_travail(*travail_iterator)){
	  travail_iterator++;
	}
	if(travail_iterator != travail.end()){
	  list<Tache> ltmp = creation_liste_taches_prod(order.get_produit(),order.get_debut_livraison(),order.get_fin_livraison());
	  int tmp_creation = ltmp.begin()->get_creation();
	  travail_iterator->splice(travail_iterator->end(), ltmp);
	  travail_iterator->begin()->set_creation(tmp_creation + 30);
	}
      }
    }
  }
}

//renvoie la tâche qui le plus grand ratio 
list<list<Tache> >::iterator max_ratio(list<list<Tache> > &travail){
  list<list<Tache> >::iterator travail_iterator;
  list<list<Tache> >::iterator tmp = travail.begin();
  for(travail_iterator = travail.begin(); travail_iterator != travail.end(); travail_iterator++){
    if(travail_iterator->begin()->get_ratio() > tmp->begin()->get_ratio())
      tmp = travail_iterator;
  }
  return tmp;
}

//calcule le ratio de chaque première tâche de chaque liste de tâche
void calcul_ratio(list<list<Tache> > &travail, int temps){
  list<list<Tache> >::iterator t_it;
  for(t_it = travail.begin(); t_it != travail.end(); t_it++){
    t_it->begin()->set_ratio(0);
    if(t_it->begin()->get_intitule() == "Destocker") {
      if(t_it->begin()->dans_les_temps(temps)) {t_it->begin()->set_ratio(100);} 
      else {t_it->begin()->set_ratio(0);}
    }
    else{
      if((t_it->size() == 1) && (t_it->begin()->get_intitule() == "Decapsuler")){
	t_it->begin()->set_ratio(0.1);
      }
      else{
	t_it->begin()->set_ratio(t_it->begin()->point_par_produit());
      }
    }
    if(t_it->begin()->get_en_traitement()){
      t_it->begin()->set_ratio(0);
    }
    t_it->begin()->set_ratio(t_it->begin()->get_ratio() / t_it->begin()->get_creation());
  }
}			

//affiche des infos sur la structure contenant les tâches à réaliser
void get_info_liste_de_liste(list<list<Tache> > travail){
list<list<Tache> >::iterator wit;
  int compteur = 1; 
  for(wit = travail.begin(); wit != travail.end(); wit++){
    cout << "taille de la liste " << compteur << " = " << wit->size() <<
        " | ratio = "<< wit->begin()->get_ratio() <<
        " | intitule première tâche = " << wit->begin()->get_intitule()  << 
        " " << wit->begin()->get_parametre() <<  
        " debut_livr : " << wit->begin()->get_debut_livraison() <<
        " fin_livr : " << wit->begin()->get_fin_livraison() << endl;
    compteur++;
  }
}			
			
//parcoure la liste d'ordres reçus par la Refbox
