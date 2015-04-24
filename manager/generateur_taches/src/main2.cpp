#include <list> 
#include <iostream>
#include <ros/ros.h>

#include "stockage.h"
#include "action.h"
#include "tache.h"
#include "listetaches.h"
#include "manager_msg/order.h"
#include "travail.h"
#include "robot.h"
#include "produit.h"
#include "machine.h"
#include "travailphase.h"
#include "etatdujeu.h"

#include "comm_msg/GameState.h" //a changer en comm_msg plus tard

using namespace std;
using namespace manager_msg;

int main(int argc, char **argv) {
  
  /*** INITIALISATION ***/
  
  ros::init(argc, argv, "action_node");
  Action action_exec;
  Etatdujeu etat_jeu;
  ros::Rate loop_rate(1);
  
  int cap_dispo = 0,storage =0, id=0, cpt_order = 0, k=0; 
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
  couleurs.push_back(10);
  couleurs.push_back(20);
  Produit product(0,couleurs);
  Ordre order(product,20,30,2,false);
  
  Machine tab_machine[6];
  
  bool take[3] = {false,false,false};
  
  double t0 = ros::Time::now().toSec();
  double temps = t0;
  
  int cpt_zone = 0;
  
  /***FONCTION PRINCIPALE ***/
    
    
  while(ros::ok() && !work.empty()) {
    // il y a trois robots
    for(int j=0; j<3; j++){
      temps = ros::Time::now().toSec() - t0;
      cout << "temps en sec = " << temps << endl;
      action_exec.update_robot(tabrobot);
      //mettre a jour les infos envoyees par la refbox
      if(!tabrobot[j].get_occupe() && cpt_zone<12){
        if(etat_jeu.get_phase()==GameState::EXPLORATION /*&& !exploration_finie(tab_machine)*/){
          travail_phase_exploration(tab_machine,tabrobot,cpt_order,j);  
        }
        /*if(etat_jeu.get_phase()==GameState::PRODUCTION && !work.empty()){
          travail_phase_production(work,tab_machine,tabrobot,tab_stock,take,cpt_order,j,cap_dispo,storage,order,temps);
          get_info_liste_de_liste(work);
        }*/
      }
    ros::spinOnce();
    loop_rate.sleep();
    }
  }
  return 0;
} 
