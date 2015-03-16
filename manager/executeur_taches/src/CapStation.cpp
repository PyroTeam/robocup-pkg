#include <string>

#include "CapStation.h"

/* Constructeur */
CapStation::CapStation(){
  m_type = "CapStation";
  m_capNoir = 0;
  m_capGris = 0;
}

/* Destructeur */
CapStation::~CapStation(){}

/* Fonction Virtuelle */
void CapStation::FonctionVirtuelle(){}

/* Méthodes */
int CapStation::getCapGris(){
  return m_capGris;
}
int CapStation::getCapNoir(){
  return m_capNoir;
}
void CapStation::majNoir(int nbNoir){
  m_capNoir = nbNoir;
}
void CapStation::majGris(int nbGris){
  m_capGris = nbGris;
}
