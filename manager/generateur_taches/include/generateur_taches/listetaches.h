#ifndef LISTETACHES_H
#define LISTETACHES_H

#include <list>
#include "tache.h"

std::list<Tache> creation_liste_taches_prod(int produit);
std::list<Tache> creation_liste_taches_act(int action,int produit);

#endif
