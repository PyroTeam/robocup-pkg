#include <list> 
#include <iostream>
#include <ros/ros.h>

#include "stockage.h"
#include "action.h"
#include "tache.h"
#include "listetaches.h"
#include "ordre.h"
#include "travail.h"
#include "robot.h"
#include "produit.h"
#include "srvorder.h"
#include "tableaustockage.h"

#include "manager_msg/order.h"
#include "manager_msg/activity.h"

using namespace std;
using namespace manager_msg;

int main(int argc, char **argv) {
  
  /*** INITIALISATION ***/
  
  ros::init(argc, argv, "action_node");
  Action action_exec;
  ros::Rate loop_rate(1);
  
  int temps = 5,cap_dispo = 0,storage =0, id=0; 
  Stockage tab_stock[6];
  
  Robot tabrobot[3];
  vector<int> black(1,10);
  Produit action(0,black);
  
  list< list<Tache> > work;
  for(int i=0;i<6;i++){
    work.push_back(creation_liste_taches_act(7,action,0,0));
  } 
  get_info_liste_de_liste(work);
  cout <<""<<endl;
  
  vector<int> couleurs;
  couleurs.push_back(0);
  couleurs.push_back(1);
  Produit product(0,couleurs);
  Ordre order(product,100,150,2,false);
  int cpt_order = 0;
  /***FONCTION PRINCIPALE ***/
    
  //mettre a jour l'etat des robot, les ordres  
  while(ros::ok() && !work.empty()) {
    // il y a trois robots
    for(int j=0; j<3; j++){
      action_exec.update_robot(tabrobot);  
      if(!work.empty()) {      
        //si le robot ne fait rien on lui donne du travail
        if(!tabrobot[j].get_occupe()){      
	  calcul_ratio(work,temps);
	  get_info_liste_de_liste(work);
	  list<list<Tache> >::iterator it = max_ratio(work);
	  tache_particuliere_travail(it,temps,cap_dispo,storage);
	  if(it->begin()->get_intitule() == orderRequest::DESTOCK)
	    id=trouver_id(tab_stock,it->begin()->get_debut_livraison(), it->begin()->get_fin_livraison());
	  Srvorder srv(ros::Time::now(),cpt_order,j,it->begin()->get_intitule(),it->begin()->get_parametre(),0);
	  cout <<"Robot n°"<<j+1<<" tâche: "<<it->begin()->get_intitule()<<" parametre: "<<it->begin()->get_parametre()
	       <<" id: " << srv.get_id()<<"\n"<<endl;
	  cpt_order++;
	  if(srv.get_accepted()){ 
	    it->begin()->set_en_traitement(true);
	    if((it->begin()->get_intitule() == orderRequest::DELIVER) && (it->begin()->get_parametre() == orderRequest::STOCK))
	      Stockage stock(it->begin()->get_produit(),it->begin()->get_debut_livraison(),it->begin()->get_fin_livraison(),srv.get_id());
	    //il faut encore prendre en compte de faire un TAKE avant de faire un PUT 
	    nettoyage_travail(work,it);
	  }
        }
        //on prend en compte les ordres de la refbox
        rajout_dans_travail(work,order,cap_dispo);
        order.set_quantite(0);
        id=0;
      }
    ros::spinOnce();
    loop_rate.sleep();
    }
  }
  return 0;
} 
