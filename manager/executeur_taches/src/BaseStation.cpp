#include <string>
#include "BaseStation.h"

/* Constructeur */
BaseStation::BaseStation(){
  m_type = "BaseStation";   
  m_baseRouge = 0;
  m_baseNoir = 0;
  m_baseGris = 0;
}

/* Destructeur */
BaseStation::~BaseStation(){}

/* Fonction Virtuelle */
void BaseStation::FonctionVirtuelle(){}

/* Méthodes */
int BaseStation::getBaseRouge(){
  return m_baseRouge;
}
int BaseStation::getBaseNoir(){
  return m_baseNoir;
}
int BaseStation::getBaseGris(){
  return m_baseGris;
}
void BaseStation::majRouge(int nbRouge){
  m_baseRouge = nbRouge;
}
void BaseStation::majNoir(int nbNoir){
  m_baseNoir = nbNoir;
}
void BaseStation::majGris(int nbGris){
  m_baseGris = nbGris;
}