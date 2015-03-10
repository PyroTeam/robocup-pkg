#include <cstdlib>
#include <list>
#include "tache.h"
#include "listetaches.h"


using namespace std;

list<Tache> creation_liste_taches_prod(int produit) {
  list<Tache> liste;
  switch(produit){
  case 0:
    liste.push_back(Tache("ChercherBase",produit,120,150,90,1));
    liste.push_back(Tache("MettreCaps",  produit,60,75,60,1));
    liste.push_back(Tache("Livrer",      produit,60,75,30,1));
    break;
  case 1:
    break;
  case 2:
    break;
  case 3:
    break;
  default:
    exit(-1);
  }
  return liste;
}

list<Tache> creation_liste_taches_act(int action,int produit) {
  list<Tache> liste;
  switch(action){
  case 0:
    liste.push_back(Tache("Decapsuler",produit,0,0,30,1));
    break;
  case 1:
    liste.push_back(Tache("Stocker",   produit,0,0,30,10));
    break;
  default:
    exit(-1);
  }
  return liste;
}

bool decapsuler_dans_travail(list<Tache> liste){
  bool tmp = false;
  if(liste.begin()->get_intitule() == "Decapsuler")
    tmp =true;
  return tmp;
}
