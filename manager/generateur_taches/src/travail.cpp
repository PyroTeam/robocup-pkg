#include <list>
#include "travail.h"
#include "tache.h"
#include "listetaches.h"
#include "refboxcomm.h"

using namespace std;

// verifie si la demande de la RefBox a déjà prise en compte
bool deja_dans_travail(list<list<Tache> > travail, Refboxcomm refbox){
  bool tmp = false;
  list<list<Tache> >::iterator travail_iterator;
  for(travail_iterator = travail.begin(); travail_iterator != travail.end(); travail_iterator++){
    if( (refbox.get_produit() == travail_iterator->begin()->get_produit()) && (refbox.get_debut_livraison() == travail_iterator->begin()->get_debut_livraison()) )
      tmp =true;
  }
  return tmp;
}

// rajoute les nouvelles listes de tâches à faire éxecuter 
void rajout_dans_travail(list<list<Tache> > &travail, Refboxcomm refbox,int cap_dispo){
  if(!deja_dans_travail(travail,refbox)){
    for(int i=0; i< refbox.get_quantite_restante(); i++){
      if(cap_dispo > 0){
	travail.push_back(creation_liste_taches_prod(refbox.get_produit()));
	cap_dispo --;
      }
      else{
	list<list<Tache> >::iterator travail_iterator;
	travail_iterator = travail.begin();
	while(travail_iterator != travail.end() && decapsuler_dans_travail(*travail_iterator)==false )
	  travail_iterator++;
	if(travail_iterator != travail.end()){
	  list<Tache> ltmp = creation_liste_taches_prod(refbox.get_produit());
	    travail_iterator->splice(travail_iterator->end(), ltmp);
	  }
      }
    }
  }
}

//renvoie la tâche qui le plus grand ratio 
Tache max_ratio(list<list<Tache> > travail){
  list<list<Tache> >::iterator travail_iterator;
  list<list<Tache> >::iterator tmp = travail.begin();
  for(travail_iterator = travail.begin(); travail_iterator != travail.end(); travail_iterator++){
    if(travail_iterator->begin()->get_ratio() > tmp->begin()->get_ratio())
      tmp = travail_iterator;
  }
  return tmp->front();
}

