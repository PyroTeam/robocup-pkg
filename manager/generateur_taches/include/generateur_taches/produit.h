#ifndef PRODUIT_H
#define PRODUIT_H

#include <vector>

class Produit{

 public:
  Produit(int nbr_ring,std::vector<std::string> parametres):m_parametres(parametres){
    m_nbr_ring=nbr_ring;
  }
  
  void set_nbr_ring(int nbr_ring){m_nbr_ring=nbr_ring;}
  int get_nbr_ring(){return m_nbr_ring;}
  void set_parametres(std::vector<std::string> parametres){m_parametres=parametres;}
  std::vector<std::string> get_parametres(){return m_parametres;}
  
  std::string get_param(int i);
        	  
 private:
  int m_nbr_ring;
  std::vector<std::string> m_parametres;
  
};
#endif
