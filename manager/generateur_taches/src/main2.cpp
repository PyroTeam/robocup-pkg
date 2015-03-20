#include <set>
#include <list> 
#include <iostream>
#include <ros/ros.h>

#include "action.h"
#include "tache.h"
#include "listetaches.h"
#include "ordre.h"
#include "travail.h"
#include "robot.h"
#include "produit.h"

#include "manager_msg/ordre.h"

using namespace std;

int main(int argc, char **argv) {
  
  /*** INITIALISATION ***/
  
  ros::init(argc, argv, "action_node");
  Action action_exec;
  ros::Rate loop_rate(10);
  
  int temps = 5,cap_dispo = 0,storage =0; 
  
  Robot tabrobot[3];
  vector<string> black(1,"noir");
  Produit action(0,black);
  
  list< list<Tache> > work;
  for(int i=0;i<6;i++){
    work.push_back(creation_liste_taches_act("Decapsuler",action,0,0));
  } 
  get_info_liste_de_liste(work);
  cout <<""<<endl;
  
  vector<string> couleurs;
  couleurs.push_back("rouge");
  couleurs.push_back("noir");
  Produit product(0,couleurs);
  Ordre order(product,100,150,2,false);
  
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
	  //utiliser service pour donner ordre a executeur de taches
	  //recevoir où c'est mis ou alors comm entre exec de taches pour que chacun sache où c'est stocké		
	  //réaliser traitement de la reponse du service pour savoir ce que fait le robotino
	  cout <<"Robot n°"<<j+1<<" fait la tâche : "<<it->begin()->get_intitule()<<" "<<it->begin()->get_parametre()<<"\n"<<endl;
	  //tabrobot[j].set_occupe(true);
	  nettoyage_travail(work,it);
	  //it->begin()->set_en_traitement(true);
        }
        //on prend en compte les ordres de la refbox
        rajout_dans_travail(work,order,cap_dispo);
        order.set_quantite(0);
      }
    ros::spinOnce();
    loop_rate.sleep();
    }
  }
  return 0;
} 
