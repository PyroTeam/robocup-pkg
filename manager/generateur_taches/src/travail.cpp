#include "travail.h"
#include "tache.h"
#include "listetaches.h"
#include "refboxcomm.h"

using namespace std;

bool deja_dans_travail(list<list<Tache> > travail, Refboxcomm refbox){
  bool tmp = false;
  list<list<Tache> >::iterator travail_iterator;
  for(travail_iterator = travail.begin(); travail_iterator != travail.end(); travail_iterator++){
    if( (refbox.get_produit() == (*travail_iterator).begin().get_produit()) && (refbox.get_debut_livraison() == (*travail_iterator).begin().get_debut_livraison()) )
      tmp =true;
  }
  return tmp;
}


void rajout_dans_travail(list<list<Tache> > travail, Refboxcomm refbox,int cap_dispo){
  if(deja_dans_travail(travail,refbox)){
    for(int i=0; i< refbox.get_quantite_restante(); i++){
      if(cap_dispo > 0){
	travail.push_back(creation_liste_taches_prod(0));
	cap_dispo --;
      }
      else{
	list<list<Tache> >::iterator travail_iterator;
	travail_iterator = travail.begin();
	while(travail_iterator != travail.end() && decapsuler_dans_travail(*travail_iterator)==false )
	  travail_iterator++;
	if(travail_iterator != travail.end())
	  (*travail_iterator).push_back(creation_liste_taches_prod(0));
      }
    }
  }
}

Tache max_ratio(list<list<Tache> > travail){
  Tache tmp = travail.begin().begin();
  list<list<Tache> >::iterator travail_iterator;
  for(travail_iterator = travail.begin(); travail_iterator != travail.end(); travail_iterator++){
    if((*travail_iterator).begin().get_ratio() > tmp.get_ratio())
      tmp = (*travail_iterator).begin();
  }
  return tmp;
}
