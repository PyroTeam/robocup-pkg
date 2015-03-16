/* Classe abstraite qui permet d'aller vers une machine */

#include <string>

#include "Machine.h"
#include "geometry_msgs/Pose2D.h"

/* Constructeur */
Machine::Machine(){
  m_centreMachine.x = 0.0;
  m_centreMachine.y = 0.0;
  m_centreMachine.theta = 0.0; 
  m_entreeMachine.x = 0.0;
  m_entreeMachine.y = 0.0;
  m_entreeMachine.theta = 0.0; 
  m_sortieMachine.x = 0.0;
  m_sortieMachine.y = 0.0;
  m_sortieMachine.theta = 0.0; 
}

/* Destructeur */
Machine::~Machine(){}

/* Méthodes */
std::string Machine::getType(){
  return m_type;
}

geometry_msgs::Pose2D Machine::getCentreMachine(){
  return m_centreMachine;
}

geometry_msgs::Pose2D Machine::getEntreeMachine(){
  return m_entreeMachine;
}

geometry_msgs::Pose2D Machine::getSortieMachine(){
  return m_sortieMachine;
}

void Machine::majEntree(geometry_msgs::Pose2D point){
  m_entreeMachine = point;
}

void Machine::majSortie(geometry_msgs::Pose2D point){
  m_sortieMachine = point;
}

void Machine::AllerMachineEntree(){
  // refBoxCom(Robotinos)
  // navigation (m_entreeMachine);
  // Approche Finale
}
