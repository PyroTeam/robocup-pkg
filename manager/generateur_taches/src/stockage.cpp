#include "stockage.h"

Stockage::Stockage(int produit,int debut,int fin,int machine,int espace){
  m_produit=produit;
  m_debut_livraison=debut;
  m_fin_livraison=fin;
  m_machine=machine;
  m_espace=espace;
}


