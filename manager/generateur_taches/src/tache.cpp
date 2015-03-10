#include <string>
#include <iostream>
#include <list>
#include <cstdlib>
#include "tache.h"

using namespace std;


Tache::Tache(string inti,int prod,int deb,int fin,int crea,float rat){
  m_intitule = inti;
  m_produit = prod;
  m_debut_livraison = deb;
  m_fin_livraison = fin;
  m_creation = crea;
  m_ratio = rat;
}

int Tache::point_par_produit(){
  if(m_produit==0) return 1;
  if(m_produit==1) return 3;
  if(m_produit==2) return 6;
  if(m_produit==3) return 10;
}

bool Tache::dans_les_temps(int temps){
  if((((m_debut_livraison - temps) <= 0) && ((m_fin_livraison - temps) > 0)) || (temps < 14*60))
    return true;
  else
    return false;
}
