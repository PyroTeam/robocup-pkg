#include "stockage.h"
#include "tableaustockage.h"

using namespace std;

bool produit_en_stock(Stockage tab_stock[]){
  bool tmp = false;
  for(int i=0;i<TAILLE;i++){
    if((tab_stock[i].get_produit() != 0) && (tab_stock[i].get_produit() != 10))
      tmp = true;
  }
  return tmp;
}


int trouver_id(Stockage tab_stock[],int debut, int fin){
        for(int i=0; i<TAILLE; i++){
                if((tab_stock[i].get_debut_livraison() == debut) && (tab_stock[i].get_fin_livraison() == fin))
                        return tab_stock[i].get_id();
        }
        return -3;
}
