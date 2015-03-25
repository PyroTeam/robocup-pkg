#include "travailphase.h"
#include "tabmachine.h"
#include "machine.h"
#include "robot.h"
#include "srvorder.h"
#include "travail.h"
#include "ordre.h"
#include "stockage.h"
#include "tableaustockage.h"

#include <ros/ros.h>
#include <list> 
#include <iostream>

#include "manager_msg/order.h"

using namespace manager_msg;
using namespace std;

void travail_phase_exploration(Machine (&tab_machine)[6], Robot (&tabrobot)[3],int &cpt_order, int robot){  
  //update_zone(tab_machine,tabrobot); //trouver une maniere efficace d'attribuer aux machines un robot
  int k=0;
  while((k<NBR_MACHINES) && (tab_machine[k].get_traite()) && (robot != tab_machine[k].get_robot())) {
    k++;
    }
  Srvorder srvexplo(ros::Time::now(),cpt_order,robot+1,orderRequest::DISCOVER,orderRequest::NONE,k+1);
  cout <<"Robot n°"<<robot<<" execute la tache :DISCOVER sur la machine n°"<<k<<endl;
  cpt_order++;
  if(srvexplo.get_accepted()){
    tab_machine[k].set_traite(true);
    tabrobot[robot].set_occupe(true);
  }
}


void travail_phase_production(list<list<Tache> > &work, Machine (&tab_machine)[6], Robot (&tabrobot)[3],Stockage (&tab_stock)[6],int &cpt_order, int robot, int temps, int &cap_dispo, int &storage,Ordre &order){ 
  if(!tabrobot[robot].get_occupe()){      
    int id = 0;
    calcul_ratio(work,temps);
    get_info_liste_de_liste(work);
    list<list<Tache> >::iterator it = max_ratio(work);
    tache_particuliere_travail(it,temps,cap_dispo,storage);
    if(it->begin()->get_intitule() == orderRequest::DESTOCK){
      id=trouver_id(tab_stock,it->begin()->get_debut_livraison(), it->begin()->get_fin_livraison());
    }
    Srvorder srv(ros::Time::now(),cpt_order,robot+1,it->begin()->get_intitule(),it->begin()->get_parametre(),0);
    cout <<"Robot n°"<<robot+1<<" tâche: "<<it->begin()->get_intitule()<<" parametre: "<<it->begin()->get_parametre()
         <<" id: " << srv.get_id()<<"\n"<<endl;
    cpt_order++; 
    if(1/*srv.get_accepted()*/){   //pour l'instant reponse du service non faite 
      it->begin()->set_en_traitement(true);
      if((it->begin()->get_intitule() == orderRequest::DELIVER) && (it->begin()->get_parametre() == orderRequest::STOCK)){
        Stockage stock(it->begin()->get_produit(),it->begin()->get_debut_livraison(),it->begin()->get_fin_livraison(),srv.get_id());
      }
      //il faut encore prendre en compte de faire un TAKE avant de faire un PUT 
      nettoyage_travail(work,it);
    }
    //on prend en compte les ordres de la refbox
    rajout_dans_travail(work,order,cap_dispo);
    order.set_quantite(0);
    id=0;
  }
}  
