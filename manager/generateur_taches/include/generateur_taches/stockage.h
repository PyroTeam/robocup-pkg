#ifndef STOCKAGE_H
#define STOCKAGE_H

class Stockage{

 public:
  Stockage(int produit,int debut,int fin,int machine,int espace);

  void set_produit(int prod){m_produit=prod;}
  int get_produit(){return m_produit;}
  void set_debut_livraison(int deb){m_debut_livraison=deb;}
  int get_debut_livraison(){return m_debut_livraison;}
  void set_fin_livraison(int fin){m_fin_livraison=fin;}
  int get_fin_livraison(){return m_fin_livraison;}
  void set_machine(int machine){m_machine=machine;}
  int get_machine(){return m_machine;}
  void set_espace(int espace){m_espace=espace;}
  int get_espace(){return m_espace;}
  
 private:
  int m_produit; //si produit = 10 alors il n'y aucun produit
  int m_debut_livraison;
  int m_fin_livraison;
  int m_machine; // Cap station 1 ou cap station 2
  int m_espace; //espace 1 2 ou 3

};

#endif
