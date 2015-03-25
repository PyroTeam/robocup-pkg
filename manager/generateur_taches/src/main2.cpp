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
#include "machine.h"
#include "tabmachine.h"
#include "travailphase.h"

#include "manager_msg/order.h"
#include "manager_msg/activity.h"

using namespace std;
using namespace manager_msg;

int main(int argc, char **argv) {
  
  /*** INITIALISATION ***/
  
  ros::init(argc, argv, "action_node");
  Action action_exec;
  ros::Rate loop_rate(1);
  
  int temps = 5,cap_dispo = 0,storage =0, id=0, cpt_order = 0, k=0; 
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
  
  Machine tab_machine[6];
  
  
  
  /***FONCTION PRINCIPALE ***/
    
    
  while(ros::ok() && !work.empty()) {
    // il y a trois robots
    for(int j=0; j<3; j++){
      action_exec.update_robot(tabrobot);
      //mettre a jour les infos envoyees par la refbox
      
      if(1/* etat du jeu=phase d exploration*/    && !exploration_finie(tab_machine)){
        travail_phase_exploration(tab_machine,tabrobot,cpt_order,j);  
      }
      
      if(0/* etat du jeu=phase de production*/    && !work.empty()){
        travail_phase_production(work,tab_machine,tabrobot,tab_stock,cpt_order,j,temps,cap_dispo,storage,order);
      }
    
    ros::spinOnce();
    loop_rate.sleep();
    }
  }
  return 0;
} 
