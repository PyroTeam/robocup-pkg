#ifndef LISTETACHES_H
#define LISTETACHES_H

#include <list>
#include "tache.h"

std::list<Tache> creation_liste_taches_prod(int produit,int debut_livraison,int fin_livraison);
std::list<Tache> creation_liste_taches_act(int action,int produit,int debut_livraison,int fin_livraison);
bool decapsuler_dans_travail(std::list<Tache> liste);

#endif
