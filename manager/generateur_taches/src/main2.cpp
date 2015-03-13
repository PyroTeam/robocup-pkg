#include <list> 
#include <iostream>
 
#include "tache.h"
#include "listetaches.h"
#include "refboxcomm.h"
#include "travail.h"
#include "robot.h"

using namespace std;

int main() {
  /*** INITIALISATION ***/
  int temps = 5; 
  int cap_dispo = 0;
  int storage =0;
  Robot tabrobot[3];
  list< list<Tache> > work;
  for(int i=0;i<6;i++){
    work.push_back(creation_liste_taches_act(0,0,0,0));
  } 
  get_info_liste_de_liste(work);
  cout <<""<<endl;
  int compteur = 0;
  Refboxcomm refbox(0,100,150,0);
  
  /***FONCTION PRINCIPALE ***/
  while(compteur !=10){
    // il y a trois robots
    for(int j=0; j<3; j++){       
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
	if(it->begin()->get_intitule() == "Stocker"){     
	  if(storage > 0){
	    storage --;
	  }
	  else{
	    cerr << "PLUS D ESPACE DE STOCKAGE DISPONIBLE !!!" << endl;
	  }
	}  			
	cout <<"Robot n°"<<j+1<<" fait la tâche : "<<it->begin()->get_intitule() <<endl;
	tabrobot[j].set_occupe(true);
	it->pop_front();
	//on supprime les listes vides
	if(it->empty()){
	  work.erase(it);
	}
	else
	  it->begin()->set_en_traitement(true);
	cout << " Taille work après réalisation de la tâche : "<< work.size() <<endl;
      }
      if(compteur==0) refbox.set_quantite(2);
      if(compteur==2) refbox.set_quantite(0);
      //on prend en compte les ordres de la refbox
      rajout_dans_travail(work,refbox,cap_dispo); 
    }
    if(compteur == 2) {tabrobot[0].set_occupe(false); }
    if(compteur == 5) {tabrobot[2].set_occupe(false); }
    if(compteur == 7) {tabrobot[1].set_occupe(false); } 
    compteur++;
  }
  return 0;
} 
