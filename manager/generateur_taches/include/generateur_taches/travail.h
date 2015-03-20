#ifndef TRAVAIL_H
#define TRAVAIL_H

#include "tache.h"
#include "ordre.h"

bool deja_dans_travail(std::list<std::list<Tache> > travail, Ordre order);
std::list<std::list<Tache> >::iterator max_ratio(std::list<std::list<Tache> > &travail);
void rajout_dans_travail(std::list<std::list<Tache> > &travail, Ordre &order, int &cap_dispo);
void calcul_ratio(std::list<std::list<Tache> > &travail, int temps);
void get_info_liste_de_liste(std::list<std::list<Tache> > travail);
void nettoyage_travail(std::list<std::list<Tache> > &travail,std::list<std::list<Tache> >::iterator &it);
void tache_particuliere_travail(std::list<std::list<Tache> > ::iterator &it,int temps, int &cap_dispo, int &storage);

#endif
