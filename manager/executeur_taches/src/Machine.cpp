
/* Classe abstraite qui permet d'aller vers une machine */

#include <string>

#include "Machine.h"
#include "geometry_msgs/Pose2D.h"

/* Constructeur */
Machine::Machine(){
  m_centerMachine.x = 0.0;
  m_centerMachine.y = 0.0;
  m_centerMachine.theta = 0.0; 
  m_entryMachine.x = 0.0;
  m_entryMachine.y = 0.0;
  m_entryMachine.theta = 0.0; 
  m_exitMachine.x = 0.0;
  m_exitMachine.y = 0.0;
  m_exitMachine.theta = 0.0; 
}

/* Destructeur */
Machine::~Machine(){}

/* Méthodes */
std::string Machine::getType(){
  return m_type;
}

geometry_msgs::Pose2D Machine::getCenterMachine(){
  return m_centerMachine;
}

geometry_msgs::Pose2D Machine::getEntryMachine(){
  return m_entryMachine;
}

geometry_msgs::Pose2D Machine::getExitMachine(){
  return m_exitMachine;
}

void Machine::majEntry(geometry_msgs::Pose2D point){
  m_entryMachine = point;
}

void Machine::majExit(geometry_msgs::Pose2D point){
  m_exitMachine = point;
}

manager_msg::activity Machine::msgToGT(int n_robot, int stateOfOrder, int machine, int n_order) // A Verifier 
{
       manager_msg::activity msg;  
       msg.nb_robot = n_robot;
       msg.state = stateOfOrder; 
       msg.machine_used = machine; 
       msg.nb_order = n_order;
       return msg;

}

/* Fonction abstraite qui permet d'aller vers une machine (Point centre/entree/sortie d'une machine) */
void Machine::goTo(geometry_msgs::Pose2D pt_dest){
  ROS_INFO("going to the point : x %lf - y %lf - theta %lf",pt_dest.x,pt_dest.y,pt_dest.theta);
  // refBoxCom(Robotinos)
  // SupervisionChemin(pt_dest);
  // Approche Finale
}

/* Fonction qui permet de prendre un produit */
//void Machine::take( ){
  // Approche Finale
  // Pince => prend le produit 
  // refBoxCom(Communication machine)
//}

/* Fonction qui permet de deposer un produit */
//void Machine::let( ){
  // Approche Finale
  // Pince => depose le produit 
  // refBoxCom(Communication machine)
//}

void Machine::readlights(){
 
  ROS_INFO(" Starting exploring the lights ");
  FeuClientAction f_c;
  f_c.lightsStates(); 
}

void Machine::startFinalAp(int8_t machineType, int8_t machineSide){
  FinalApproachingClient fa_c;
  fa_c.starting(machineType,machineSide);
}