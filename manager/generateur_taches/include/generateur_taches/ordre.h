#ifndef ORDRE_H
#define ORDRE_H

#include <vector>
#include "produit.h"

class Ordre{

 public:
  Ordre(Produit produit,int debut_livraison,int fin_livraison,int quantite,bool traite):m_produit(produit){
    
    m_debut_livraison=debut_livraison;
    m_fin_livraison=fin_livraison;
    m_quantite=quantite;
    m_traite=traite;
    
  }
  
  void set_produit(Produit prod){m_produit=prod;}
  Produit get_produit(){return m_produit;}
  void set_debut_livraison(int deb){m_debut_livraison=deb;}
  int get_debut_livraison(){return m_debut_livraison;}
  void set_fin_livraison(int fin){m_fin_livraison=fin;}
  int get_fin_livraison(){return m_fin_livraison;}
  void set_quantite(int quan){m_quantite=quan;}
  int get_quantite(){return m_quantite;}
  void set_traite(bool traite){m_traite=traite;}
  bool get_traite(){return m_traite;}
	  
 private:
  Produit m_produit;
  int m_debut_livraison;
  int m_fin_livraison;
  int m_quantite;
  bool m_traite;
};
#endif
