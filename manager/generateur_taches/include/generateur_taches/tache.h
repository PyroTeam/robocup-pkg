#ifndef TACHE_H
#define TACHE_H

#include <string>
#include <list>

class Tache{

 public:
  Tache(std::string inti, std::string parametre,int prod, int deb, int fin, int crea, float rat,bool en_traitement);

  void set_intitule(std::string inti){m_intitule=inti;}
  std::string get_intitule(){return m_intitule;}
  void set_parametre(std::string parametre){m_parametre=parametre;}
  std::string get_parametre(){return m_parametre;}
  void set_produit(int prod){m_produit=prod;}
  int get_produit(){return m_produit;}
  void set_debut_livraison(int deb){m_debut_livraison=deb;}
  int get_debut_livraison(){return m_debut_livraison;}
  void set_fin_livraison(int fin){m_fin_livraison=fin;}
  int get_fin_livraison(){return m_fin_livraison;}
  void set_creation(int crea){m_creation=crea;}
  int get_creation(){return m_creation;}
  void set_ratio(float rat){m_ratio=rat;}
  float get_ratio(){return m_ratio;}
  void set_en_traitement(bool en_traitement){m_en_traitement=en_traitement;}
  bool get_en_traitement(){return m_en_traitement;}  
  
  int point_par_produit();
  bool dans_les_temps(int temps);
  
 private:
  std::string m_intitule;
  std::string m_parametre;
  int m_produit; 
  int m_debut_livraison;
  int m_fin_livraison;
  int m_creation; //temps de creation restant
  float m_ratio;
  bool m_en_traitement;
};

#endif

