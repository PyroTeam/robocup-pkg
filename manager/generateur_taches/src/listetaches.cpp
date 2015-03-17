#include <cstdlib>
#include <list>
#include "tache.h"
#include "listetaches.h"
#include "produit.h"


using namespace std;

list<Tache> creation_liste_taches_prod(Produit produit,int debut_livraison,int fin_livraison) {
  list<Tache> liste;
  switch(produit.get_nbr_ring()){
  case 0:
    liste.push_back(Tache("ChercherBase", produit.get_param(0),produit.get_nbr_ring(),debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit.get_param(1),produit.get_nbr_ring(),debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       ""                  ,produit.get_nbr_ring(),debut_livraison,fin_livraison,30,1,false));
    break;
  case 1:
    liste.push_back(Tache("ChercherBase", produit.get_param(0),produit.get_nbr_ring(),debut_livraison,fin_livraison,120,1,false));
    liste.push_back(Tache("Mettrering",   produit.get_param(1),produit.get_nbr_ring(),debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit.get_param(2),produit.get_nbr_ring(),debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       ""                  ,produit.get_nbr_ring(),debut_livraison,fin_livraison,30,1,false));
    break;
  case 2:
    liste.push_back(Tache("ChercherBase", produit.get_param(0),produit.get_nbr_ring(),debut_livraison,fin_livraison,180,1,false));
    liste.push_back(Tache("Mettrering",   produit.get_param(1),produit.get_nbr_ring(),debut_livraison,fin_livraison,150,1,false));
    liste.push_back(Tache("RamenerbaseRS",produit.get_param(2),produit.get_nbr_ring(),debut_livraison,fin_livraison,120,1,false));
    liste.push_back(Tache("Mettrering",   produit.get_param(2),produit.get_nbr_ring(),debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit.get_param(3),produit.get_nbr_ring(),debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       ""                  ,produit.get_nbr_ring(),debut_livraison,fin_livraison,30,1,false));
    break;
  case 3:
    liste.push_back(Tache("ChercherBase", produit.get_param(0),produit.get_nbr_ring(),debut_livraison,fin_livraison,270,1,false));
    liste.push_back(Tache("Mettrering",   produit.get_param(1),produit.get_nbr_ring(),debut_livraison,fin_livraison,240,1,false));
    liste.push_back(Tache("RamenerbaseRS",produit.get_param(2),produit.get_nbr_ring(),debut_livraison,fin_livraison,210,1,false));
    liste.push_back(Tache("RamenerbaseRS",produit.get_param(2),produit.get_nbr_ring(),debut_livraison,fin_livraison,180,1,false));
    liste.push_back(Tache("Mettrering",   produit.get_param(2),produit.get_nbr_ring(),debut_livraison,fin_livraison,150,1,false));
    liste.push_back(Tache("RamenerbaseRS",produit.get_param(3),produit.get_nbr_ring(),debut_livraison,fin_livraison,120,1,false));
    liste.push_back(Tache("Mettrering",   produit.get_param(3),produit.get_nbr_ring(),debut_livraison,fin_livraison,90,1,false));
    liste.push_back(Tache("MettreCaps",   produit.get_param(4),produit.get_nbr_ring(),debut_livraison,fin_livraison,60,1,false));
    liste.push_back(Tache("Livrer",       ""                  ,produit.get_nbr_ring(),debut_livraison,fin_livraison,30,1,false));
    break;
  default:
    exit(-1);
  }
  return liste;
}

list<Tache> creation_liste_taches_act(string action,Produit produit,int debut_livraison,int fin_livraison) {
  list<Tache> liste;
  if(action=="Decapsuler")
    liste.push_back(Tache("Decapsuler"," ",produit.get_nbr_ring(),debut_livraison,fin_livraison,30,1,false));
  if(action=="Destocker")
    liste.push_back(Tache("Destocker" ," ",produit.get_nbr_ring(),debut_livraison,fin_livraison,30,1,false));
  if(action=="Stocker")
    liste.push_back(Tache("Stocker"   ," ",produit.get_nbr_ring(),debut_livraison,fin_livraison,30,1,false));
  return liste;
}

bool decapsuler_dans_travail(list<Tache> liste){
  bool tmp = false;
  if(liste.begin()->get_intitule() == "Decapsuler")
    tmp =true;
  return tmp;
}
