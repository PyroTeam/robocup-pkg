#include <list> 
#include <iostream> 
#include "tache.h"
#include "listetaches.h"
#include "refboxcomm.h"
#include "travail.h"

using namespace std;

int main() { 
  /*Tache task("Decapsuler",0,0,0,30,1);
  cout << task.get_intitule() << endl;
  task.set_intitule("ChercherBase");
  cout << task.get_intitule() << endl;
  cout << task.point_par_produit() << endl;
  
  list<Tache> liste=creation_liste_taches_prod(0);
  list<Tache>::iterator liste_iterator;
  for(liste_iterator = liste.begin(); liste_iterator != liste.end(); liste_iterator++)
    cout << (*liste_iterator).get_intitule() << endl;
  cout << liste.size() << endl;*/
  
  list< list<Tache> > work;
  for(int i=0;i<5;i++)
    work.push_back(creation_liste_taches_act(0,0));
  
  cout << "\n\tInitialement la liste de liste de taches a realiser est seulement constituee de cinq fois la tache Decapsuler" << endl;
  list<list<Tache> >::iterator wit;
  int compteur1 = 0; 
  for(wit = work.begin(); wit != work.end(); wit++){
  	cout << "taille de la liste " << compteur1 << " = " << (*wit).size() << " | intitule = " << wit->begin()->get_intitule() << " | ratio = "<< wit->begin()->get_ratio() << endl;
  	compteur1++;
  	}
  cout <<""<<endl;
  Refboxcomm refbox(0,100,150,2);
  cout << "\tLa refbox demande de creer deux fois le produit 0" << endl;
  cout << "\tTest de la fonction deja_dans_travail" << endl;
  if(deja_dans_travail(work, refbox))
  	cout << "les ordres de la refbox ont deja ete pris en compte" << endl;
  else if(!deja_dans_travail(work, refbox))
  	cout << "les ordres de la refbox n ont pas encore ete pris en compte" << endl;
  int cap_dispo = 1;
  cout << "\n\tSeul un cap est disponible : cap_dispo = " << cap_dispo << endl;
  cout << "\tOn va donc rajouter les nouveaux ordres dans la liste de liste de taches a realiser" <<endl;
  cout << "\tTest de la fonction rajout_dans_work" << endl;
  rajout_dans_travail(work,refbox,cap_dispo);
  int compteur2 = 0; 
  for(wit = work.begin(); wit != work.end(); wit++){
  	cout << "taille de la liste " << compteur2 << " = " << (*wit).size() << " | intitule = " << wit->begin()->get_intitule() << " | ratio = "<< wit->begin()->get_ratio() << endl;
  	compteur2++;
  	}
  cout <<"\n\tA present on va vérifier que la fonction max_ratio fonctionne" << endl;
  Tache tmp = max_ratio(work);
  cout << "La tache ayant son ratio le plus grand a pour ratio : "<< tmp.get_ratio() << " et pour intitule : " << tmp.get_intitule() << endl;
  return 0;
} 
