#ifndef REFBOXCOMM_H
#define REFBOXCOMM_H

class Refboxcomm{

 public:
  Refboxcomm(int produit,int debut_livraison,int fin_livraison,int quantite){
    m_produit=produit;
    m_debut_livraison=debut_livraison;
    m_fin_livraison=fin_livraison;
    m_quantite=quantite;
  }
  void set_produit(int prod){m_produit=prod;}
  int get_produit(){return m_produit;}
  void set_debut_livraison(int deb){m_debut_livraison=deb;}
  int get_debut_livraison(){return m_debut_livraison;}
  void set_fin_livraison(int fin){m_fin_livraison=fin;}
  int get_fin_livraison(){return m_fin_livraison;}
  void set_quantite(int quan){m_quantite=quan;}
  int get_quantite(){return m_quantite;}
	  
 private:
  int m_produit;
  int m_debut_livraison;
  int m_fin_livraison;
  int m_quantite;
};
#endif
