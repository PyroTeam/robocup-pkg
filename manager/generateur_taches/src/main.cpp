#include <list> 
#include <iostream> 
#include "tache.h"
#include "listetaches.h"
#include "refboxcomm.h"
#include "travail.h"

using namespace std;

int main() { 
  
  /*** INITIALISATION ***/
  list< list<Tache> > work;
  for(int i=0;i<5;i++){
    work.push_back(creation_liste_taches_act(0,0,0,0));
  }
  work.push_back(creation_liste_taches_act(1,0,90,140));   
  cout << "\n\tInitialement la liste de liste de taches a realiser" << endl;
  cout << "est seulement constituee de cinq fois la tache Decapsuler" << endl;
  cout << "et d une fois la tache Destocker" << endl;
  get_info_liste_de_liste(work);
  cout <<""<<endl;
  Refboxcomm refbox(0,100,150,2);
  cout << "\tLa refbox demande de creer deux fois le produit 0" << endl;
  
  /*** TEST DE LA FONCTION DEJA_DANS_TRAVAIL ***/
  cout << "\tTest de la fonction deja_dans_travail" << endl;
  if(deja_dans_travail(work, refbox))
    cout << "les ordres de la refbox ont deja ete pris en compte" << endl;
  else if(!deja_dans_travail(work, refbox))
    cout << "les ordres de la refbox n ont pas encore ete pris en compte" << endl;
  
  /*** TEST DE LA FONCTION RAJOUT_DANS_TRAVAIL ***/
  int cap_dispo = 1;
  cout << "\n\tSeul un cap est disponible : cap_dispo = " << cap_dispo << endl;
  cout << "\tOn va donc rajouter les nouveaux ordres dans la liste de liste de taches a realiser" <<endl;
  cout << "\tTest de la fonction rajout_dans_work" << endl;
  rajout_dans_travail(work,refbox,cap_dispo);
  get_info_liste_de_liste(work);
  
  /*** TEST DE LA FONCTION CALCUL_RATIO ***/
  cout << "\n\tTest de la fonction calcul_ratio" << endl;
  calcul_ratio(work,10);
  get_info_liste_de_liste(work);
  
  /*** TEST DE LA FONCTION MAX_RATIO ***/
  cout <<"\n\tA present on va vérifier que la fonction max_ratio fonctionne" << endl;
  list<list<Tache> >::iterator tmp = max_ratio(work);
  cout << "La tache ayant son ratio le plus grand a pour ratio : " << tmp->begin()->get_ratio() << endl;
  cout << " et pour intitule : "<<tmp->begin()->get_intitule()<<"\n" << endl;
  
  return 0;
} 
