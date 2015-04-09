#include <cstdlib>
#include <list>
#include "tache.h"
#include "listetaches.h"
#include "produit.h"

#include "manager_msg/order.h"  

using namespace std;
using namespace manager_msg;

list<Tache> creation_liste_taches_prod(Produit prod,int debut_livr,int fin_livr) {
  list<Tache> liste;
  switch(prod.get_nbr_ring()){
  case 0:
    liste.push_back(Tache(int(orderRequest::TAKE_BASE),prod.get_param(0), prod.get_nbr_ring(),debut_livr,fin_livr,5));
    liste.push_back(Tache(int(orderRequest::PUT_CAP),  prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,0));
    liste.push_back(Tache(int(orderRequest::TAKE_CAP), prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,60));
    liste.push_back(Tache(int(orderRequest::DELIVER),  orderRequest::NONE,prod.get_nbr_ring(),debut_livr,fin_livr,0));
    break;
  case 1:
    liste.push_back(Tache(int(orderRequest::TAKE_BASE), prod.get_param(0), prod.get_nbr_ring(),debut_livr,fin_livr,180));
    liste.push_back(Tache(int(orderRequest::PUT_RING),  prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,150));
    liste.push_back(Tache(int(orderRequest::TAKE_RING), prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,120));
    liste.push_back(Tache(int(orderRequest::PUT_CAP),   prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,90));
    liste.push_back(Tache(int(orderRequest::TAKE_CAP),  prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,60));
    liste.push_back(Tache(int(orderRequest::DELIVER),   orderRequest::NONE,prod.get_nbr_ring(),debut_livr,fin_livr,30));
    break;
  case 2:
    liste.push_back(Tache(int(orderRequest::TAKE_BASE),    prod.get_param(0), prod.get_nbr_ring(),debut_livr,fin_livr,270));
    liste.push_back(Tache(int(orderRequest::PUT_RING),     prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,240));
    liste.push_back(Tache(int(orderRequest::TAKE_RING),    prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,210));
    liste.push_back(Tache(int(orderRequest::BRING_BASE_RS),prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,180));
    liste.push_back(Tache(int(orderRequest::PUT_RING),     prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,150));
    liste.push_back(Tache(int(orderRequest::TAKE_RING),    prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,120));
    liste.push_back(Tache(int(orderRequest::PUT_CAP),      prod.get_param(3), prod.get_nbr_ring(),debut_livr,fin_livr,90));
    liste.push_back(Tache(int(orderRequest::TAKE_CAP),     prod.get_param(3), prod.get_nbr_ring(),debut_livr,fin_livr,60));
    liste.push_back(Tache(int(orderRequest::DELIVER),      orderRequest::NONE,prod.get_nbr_ring(),debut_livr,fin_livr,30));
    break;
  case 3:
    liste.push_back(Tache(int(orderRequest::TAKE_BASE),    prod.get_param(0), prod.get_nbr_ring(),debut_livr,fin_livr,390));
    liste.push_back(Tache(int(orderRequest::PUT_RING),     prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,360));
    liste.push_back(Tache(int(orderRequest::TAKE_RING),    prod.get_param(1), prod.get_nbr_ring(),debut_livr,fin_livr,330));
    liste.push_back(Tache(int(orderRequest::BRING_BASE_RS),prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,300));
    liste.push_back(Tache(int(orderRequest::BRING_BASE_RS),prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,270));
    liste.push_back(Tache(int(orderRequest::PUT_RING),     prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,240));
    liste.push_back(Tache(int(orderRequest::TAKE_RING),    prod.get_param(2), prod.get_nbr_ring(),debut_livr,fin_livr,210));
    liste.push_back(Tache(int(orderRequest::BRING_BASE_RS),prod.get_param(3), prod.get_nbr_ring(),debut_livr,fin_livr,180));
    liste.push_back(Tache(int(orderRequest::PUT_RING),     prod.get_param(3), prod.get_nbr_ring(),debut_livr,fin_livr,150));
    liste.push_back(Tache(int(orderRequest::TAKE_RING),    prod.get_param(3), prod.get_nbr_ring(),debut_livr,fin_livr,120));
    liste.push_back(Tache(int(orderRequest::PUT_CAP),      prod.get_param(4), prod.get_nbr_ring(),debut_livr,fin_livr,90));
    liste.push_back(Tache(int(orderRequest::TAKE_CAP),     prod.get_param(4), prod.get_nbr_ring(),debut_livr,fin_livr,60));
    liste.push_back(Tache(int(orderRequest::DELIVER),      orderRequest::NONE,prod.get_nbr_ring(),debut_livr,fin_livr,30));
    break;
  default:
    exit(-1);
  }
  return liste;
}

list<Tache> creation_liste_taches_act(int action,Produit prod,int debut_livr,int fin_livr) {
  list<Tache> liste;
  switch(action){
  case int(orderRequest::UNCAP): 
    liste.push_back(Tache(int(orderRequest::UNCAP),  int(orderRequest::NONE),prod.get_nbr_ring(),debut_livr,fin_livr,30));
    break;
  case int(orderRequest::DESTOCK):
    liste.push_back(Tache(int(orderRequest::DESTOCK),int(orderRequest::NONE),prod.get_nbr_ring(),debut_livr,fin_livr,30));
    break;
  case int(orderRequest::STOCK):
    liste.push_back(Tache(int(orderRequest::STOCK),  int(orderRequest::NONE),prod.get_nbr_ring(),debut_livr,fin_livr,30));
    break;
  default:
    exit(-2);
    }
  return liste;
}

bool decapsuler_dans_travail(list<Tache> liste){
  bool tmp = false;
  if(liste.begin()->get_intitule() == int(orderRequest::UNCAP))
    tmp =true;
  return tmp;
}
