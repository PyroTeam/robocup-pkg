#ifndef TRAVAIL_H
#define TRAVAIL_H

#include "tache.h"
#include "refboxcomm.h"

bool deja_dans_travail(std::list<std::list<Tache> > travail, Refboxcomm refbox);
Tache max_ratio(std::list<std::list<Tache> > travail);
void rajout_dans_travail(std::list<std::list<Tache> > travail, Refboxcomm refbox);

#endif
