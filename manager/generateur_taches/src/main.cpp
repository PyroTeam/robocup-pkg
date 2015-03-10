#include <list> 
#include <iostream> 
#include "tache.h"
#include "listetaches.h"
#include "refboxcomm.h"
#include "travail.h"

using namespace std;

int main() { 
  Tache task("Decapsuler",0,0,0,30,1);
  cout << task.get_intitule() << endl;
  task.set_intitule("ChercherBase");
  cout << task.get_intitule() << endl;
  cout << task.point_par_produit() << endl;
  
  list<Tache> liste=creation_liste_taches_prod(0);
  list<Tache>::iterator liste_iterator;
  for(liste_iterator = liste.begin(); liste_iterator != liste.end(); liste_iterator++)
    cout << (*liste_iterator).get_intitule() << endl;
  cout << liste.size() << endl;
  
  list< list<Tache> > work;
  for(int i=0;i<6;i++)
    work.push_back(creation_liste_taches_act(0,0));
  work.push_back(creation_liste_taches_prod(0));
  cout << work.size() << endl;
  Refboxcomm refbox(0,120,150,2);
  cout << deja_dans_travail(work, refbox) <<endl;
  
  return 0;
} 
