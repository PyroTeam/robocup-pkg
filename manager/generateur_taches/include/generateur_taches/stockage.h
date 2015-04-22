#ifndef STOCKAGE_H
#define STOCKAGE_H

class Stockage{

 public:
  Stockage(int produit=0,int debut=0,int fin=0,int id=0);

  void set_produit(int prod){m_produit=prod;}
  int get_produit(){return m_produit;}
  void set_debut_livraison(int deb){m_debut_livraison=deb;}
  int get_debut_livraison(){return m_debut_livraison;}
  void set_fin_livraison(int fin){m_fin_livraison=fin;}
  int get_fin_livraison(){return m_fin_livraison;}
  void set_id(int id){m_id=id;}
  int get_id(){return m_id;}
  
 private:
  int m_produit; //si produit = 10 alors il n'y aucun produit
  int m_debut_livraison;
  int m_fin_livraison;
  int m_id;

};

#endif
