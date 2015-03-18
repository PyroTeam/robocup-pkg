#include <set>
#include <list> 
#include <iostream>
 
#include "tache.h"
#include "listetaches.h"
#include "ordre.h"
#include "travail.h"
#include "robot.h"
#include "produit.h"

#include "manager_msg/ordre.h"

using namespace std;

int main() {
  /*** INITIALISATION ***/
  int temps = 5; 
  int cap_dispo = 0;
  int storage =0;
  Robot tabrobot[3];
  vector<string> nothing(1,"rien");
  vector<string> black(1,"noir");
  Produit action(0,black);
  list< list<Tache> > work;
  for(int i=0;i<6;i++){
    work.push_back(creation_liste_taches_act("Decapsuler",action,0,0));
  } 
  get_info_liste_de_liste(work);
  cout <<""<<endl;
  int compteur = 0;
  int i = 0;
  vector<string> couleurs;
  couleurs.push_back("rouge");
  couleurs.push_back("noir");
  Produit product(0,couleurs);
  Ordre order(product,100,150,2,false);
  
  /***FONCTION PRINCIPALE ***/
    
    //tant que non fin
    //mettre a jour l'etat des robot, les ordres
    
  while(!work.empty()) {
    // il y a trois robots
    for(int j=0; j<3; j++){ 
    if(!work.empty()) {      
      //si le robot ne fait rien on lui donne du travail
      if(!tabrobot[j].get_occupe()){      
	calcul_ratio(work,temps);
	get_info_liste_de_liste(work);
	list<list<Tache> >::iterator it = max_ratio(work);
	//si la seule tâche est de décapsuler dans la liste
	if((it->begin()->get_intitule() == "Decapsuler") && (it->size() == 1)){ 
	  cap_dispo ++;
	  storage ++;
	}
	if(it->begin()->get_intitule() == "Livrer"){     
	  if(it->begin()->dans_les_temps(temps)){
	    it->begin()->set_parametre("DS");
	    }
	  else
	    it->begin()->set_parametre("stock");{
	    //recevoir où c'est mis ou alors comm entre exec de taches pour que chacun sache où c'est stocké
	    Produit prod_tmp(it->begin()->get_produit(),nothing);
	    list<Tache> ltmp = creation_liste_taches_act("Destocker",prod_tmp,it->begin()->get_debut_livraison(),it->begin()->get_fin_livraison());
	    it->splice(it->end(),ltmp);
	    }
	}  			
	cout <<"Robot n°"<<j+1<<" fait la tâche : "<<it->begin()->get_intitule()<<" "<<it->begin()->get_parametre()<<"\n"<<endl;
	//tabrobot[j].set_occupe(true);
	cout<<"taille work ="<<work.size()<<endl; 
	cout << "taille it : " << it->size() << endl;               
	if(!it->empty()) it->pop_front();
	//on supprime les listes vides
	if(it->empty()){
	  work.erase(it);
	}
	else
	  //it->begin()->set_en_traitement(true);
	cout << " Taille work après réalisation de la tâche : "<< work.size() <<endl;
      }
      
      //on prend en compte les ordres de la refbox
      rajout_dans_travail(work,order,cap_dispo);
      order.set_quantite(0);
      }
    }
    
    }
  return 0;
} 
