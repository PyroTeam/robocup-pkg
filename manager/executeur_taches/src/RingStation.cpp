#include <string>

#include "RingStation.h"

/* Constructeur */
RingStation::RingStation(){
  m_type = "RingStation";
  m_ringVert = 0;
  m_ringJaune = 0;
  m_ringBleu = 0;
  m_ringOrange = 0;
}

/* Destructeur */
RingStation::~RingStation(){}

/* Fonction Virtuelle */
void RingStation::FonctionVirtuelle(){}

/* Méthodes */
int RingStation::getRingVert(){
  return m_ringVert;
}
int RingStation::getRingJaune(){
  return m_ringJaune;
}
int RingStation::getRingBleu(){
  return m_ringBleu;
}
int RingStation::getRingOrange(){
  return m_ringOrange;
}
void RingStation::majVert(int nbVert){
  m_ringVert = nbVert;
}
void RingStation::majJaune(int nbJaune){
  m_ringJaune = nbJaune;
}
void RingStation::majBleu(int nbBleu){
  m_ringBleu = nbBleu;
}
void RingStation::majOrange(int nbOrange){
  m_ringOrange = nbOrange;
}
