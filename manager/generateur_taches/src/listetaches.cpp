#include <cstdlib>
#include <list>
#include "tache.h"
#include "listetaches.h"


using namespace std;

list<Tache> creation_liste_taches_prod(int produit,int debut_livraison,int fin_livraison) {
  list<Tache> liste;
  switch(produit){
  case 0:
    liste.push_back(Tache("ChercherBase", produit,debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit,debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       produit,debut_livraison,fin_livraison,30,1,false));
    break;
  case 1:
    liste.push_back(Tache("ChercherBase", produit,debut_livraison,fin_livraison,120,1,false));
    liste.push_back(Tache("Mettrering",   produit,debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit,debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       produit,debut_livraison,fin_livraison,30,1,false));
    break;
  case 2:
    liste.push_back(Tache("ChercherBase", produit,debut_livraison,fin_livraison,180,1,false));
    liste.push_back(Tache("Mettrering",   produit,debut_livraison,fin_livraison,150,1,false));
    liste.push_back(Tache("RamenerbaseRS",produit,debut_livraison,fin_livraison,120,1,false));
    liste.push_back(Tache("Mettrering",   produit,debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit,debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       produit,debut_livraison,fin_livraison,30,1,false));
    break;
  case 3:
    liste.push_back(Tache("ChercherBase", produit,debut_livraison,fin_livraison,240,1,false));
    liste.push_back(Tache("Mettrering",   produit,debut_livraison,fin_livraison,210,1,false));
    liste.push_back(Tache("RamenerbaseRS",produit,debut_livraison,fin_livraison,180,1,false));
    liste.push_back(Tache("Mettrering",   produit,debut_livraison,fin_livraison,150,1,false));
    liste.push_back(Tache("RamenerbaseRS",produit,debut_livraison,fin_livraison,120,1,false));
    liste.push_back(Tache("Mettrering",   produit,debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit,debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       produit,debut_livraison,fin_livraison,30,1,false));
    break;
  default:
    exit(-1);
  }
  return liste;
}

list<Tache> creation_liste_taches_act(int action,int produit,int debut_livraison,int fin_livraison) {
  list<Tache> liste;
  switch(action){
  case 0:
    liste.push_back(Tache("Decapsuler",produit,0,0,30,1,false));
    break;
  case 1:
    liste.push_back(Tache("Destocker",produit,debut_livraison,fin_livraison,30,1,false));
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
