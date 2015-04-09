#include <string>
#include <iostream>
#include <list>
#include <cstdlib>
#include <ros/ros.h>

#include "tache.h"

using namespace std;


Tache::Tache(int inti,int parametre,int prod,int deb,int fin,int crea,float rat,bool en_traitement, int robot, int fin_exec){
  m_intitule = inti;
  m_parametre = parametre;
  m_produit = prod;
  m_debut_livraison = deb;
  m_fin_livraison = fin;
  m_creation = crea;
  m_ratio = rat;
  m_en_traitement = en_traitement;
  m_robot = robot;
  m_fin_execution = fin_exec;
}

int Tache::point_par_produit(){
  int tmp;
  if(m_produit==0) tmp=1;
  if(m_produit==1) tmp=3;
  if(m_produit==2) tmp=6;
  if(m_produit==3) tmp=10;
  return tmp;
}

bool Tache::dans_les_temps(double temps){
  if((((m_debut_livraison - temps) <= 0) && ((m_fin_livraison - temps) > 0)) || (temps > 14*60))
    return true;
  else
    return false;
}

