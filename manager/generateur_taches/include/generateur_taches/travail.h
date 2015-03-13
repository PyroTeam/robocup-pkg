#ifndef TRAVAIL_H
#define TRAVAIL_H

#include "tache.h"
#include "refboxcomm.h"

bool deja_dans_travail(std::list<std::list<Tache> > travail, Refboxcomm refbox);
std::list<std::list<Tache> >::iterator max_ratio(std::list<std::list<Tache> > &travail);
void rajout_dans_travail(std::list<std::list<Tache> > &travail, Refboxcomm refbox, int &cap_dispo);
void calcul_ratio(std::list<std::list<Tache> > &travail, int temps);
void get_info_liste_de_liste(std::list<std::list<Tache> > travail);

#endif
