#include <string>
#include "BaseStation.h"

/* Constructeur */
BaseStation::BaseStation(){
  m_type = "BaseStation";   
  m_redBase = 0;
  m_blackBase = 0;
  m_silverBase = 0;
}

/* Destructeur */
BaseStation::~BaseStation(){}

/* Fonction Virtuelle */
void BaseStation::FonctionVirtuelle(){}

/* Méthodes */
int BaseStation::getRedBase(){
  return m_redBase;
}
int BaseStation::getBlackBase(){
  return m_blackBase;
}
int BaseStation::getSilverBase(){
  return m_silverBase;
}
void BaseStation::majRed(int nbRouge){
  m_redBase = nbRouge;
}
void BaseStation::majBlack(int nbNoir){
  m_blackBase = nbNoir;
}
void BaseStation::majSilver(int nbArgent){
  m_silverBase= nbArgent;
}

void BaseStation::take_base(int color,int n_robot,int n_order){
     /* TOPIC Générateur de taches : infos sur l'avancement de la tache */
  manager_msg::activity msg;
  msg = msgToGT(n_robot,activity::IN_PROGRESS,activity::BS,n_order); 
  ROS_INFO("Taking a Base, color : %d", color); 

  //goTo(this.m_entryMachine);
  //Communication_RefBox(je veux une base de couleur "couleur" )
  //goTo(this.m_exitMachine);
  // while(Communication_RefBox(bs n'a terminé de livrer)) { }
}

void BaseStation::bring_base_rs(int color,int n_robot,int n_order,int machine){
     /* TOPIC Générateur de taches : infos sur l'avancement de la tache */
  manager_msg::activity msg;
  msg = msgToGT(n_robot,activity::IN_PROGRESS,activity::BS,n_order); 
  ROS_INFO("Bringing a Base to a RS"); 
  /* puis, msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); */
}