#include "travailphase.h"
#include "tabmachine.h"
#include "machine.h"
#include "robot.h"
#include "srvorder.h"
#include "travail.h"
#include "ordre.h"
#include "stockage.h"
#include "tableaustockage.h"
#include "produit.h"
#include "listetaches.h"
#include "correspondanceZE.h"

#include <ros/ros.h>
#include <list>
#include <iostream>

#include "manager_msg/order.h"

using namespace manager_msg;
using namespace std;

void travail_phase_exploration(Machine (&tab_machine)[6], Robot (&tabrobot)[3],int &cpt_order, int robot,int &cpt_zone, CorrespondanceZE &correspondanceZE){
  //update_zone(tab_machine,tabrobot); //trouver une maniere efficace d'attribuer aux machines un robot
  int cpt_machine=0;

  vector<int> zone = correspondanceZE.get_zone_utile();
  std::cout << "Taile zone : " << zone.size() << std::endl;
  if(zone.size() == 12){
		if(cpt_machine<6){
			Srvorder srvexplo(ros::Time::now(),cpt_order,robot+1,orderRequest::DISCOVER,orderRequest::NONE,zone[cpt_zone]);
            cout <<"Robot n°"<<robot+1<<" execute la tache DISCOVER sur la zone n "<<zone[cpt_zone]<<endl;
			cpt_zone++;
			cpt_order++;
			if(srvexplo.get_accepted()){ //pour l'instant reponse du service non faite
				//tab_machine[k].set_traite(true);
				cpt_machine++;
				tabrobot[robot].set_occupe(true);
			}
		}
	}
}


void travail_phase_production(list<list<Tache> > &work, Machine (&tab_machine)[6], Robot (&tabrobot)[3],Stockage (&tab_stock)[6],bool (&take)[3],int &cpt_order, int robot, int &cap_dispo, int &storage,Ordre &order, double temps){
  if(!tabrobot[robot].get_occupe()){
    int id = 0;
    taches_terminees(work,robot,temps);
    if(ratio_positif(work)){
      calcul_ratio(work,temps,robot,take);
      list<list<Tache> >::iterator it = max_ratio(work);
      tache_particuliere_travail(it,cap_dispo,storage,temps);
      if(it->begin()->get_intitule() == orderRequest::DESTOCK){
        id=trouver_id(tab_stock,it->begin()->get_debut_livraison(), it->begin()->get_fin_livraison());
      }
      Srvorder srv(ros::Time::now(),cpt_order,robot+1,it->begin()->get_intitule(),it->begin()->get_parametre(),0);
      cout <<"Robot n°"<<robot+1<<" tâche: "<<it->begin()->get_intitule()<<" parametre: "<<it->begin()->get_parametre()
           <<" id: " << srv.get_id()<<"\n"<<endl;
      cpt_order++;
      if(srv.get_accepted()){   //pour l'instant reponse du service non faite
        it->begin()->set_en_traitement(true);
        it->begin()->set_robot(robot);

        if((it->begin()->get_intitule() == orderRequest::DELIVER) && (it->begin()->get_parametre() == orderRequest::STOCK)){
          vector<int> rien(1,20);
          Produit prod_tmp(it->begin()->get_produit(),rien);
          list<Tache> ltmp = creation_liste_taches_act(int(orderRequest::DESTOCK),prod_tmp,it->begin()->get_debut_livraison(),it->begin()->get_fin_livraison());
          it->splice(it->end(),ltmp);
          Stockage stock(it->begin()->get_produit(),it->begin()->get_debut_livraison(),it->begin()->get_fin_livraison(),srv.get_id());
        }
        //il faut encore prendre en compte de faire un TAKE puis directement un PUT

        if((it->begin()->get_intitule() == 0) || (it->begin()->get_intitule() == 2) || (it->begin()->get_intitule() == 4)){
          take[robot] = true;
        }
        else{
          take[robot] = false;
        }

        nettoyage_travail(work,it,temps);
        tabrobot[robot].set_occupe(true);
      }
    }
    //on prend en compte les ordres de la refbox
    rajout_dans_travail(work,order,cap_dispo);
    order.set_quantite(0);
    id=0;
  }
}
