
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
  ROS_INFO("going to the point : x %f - y %f - theta %f",pt_dest.x,pt_dest.y,pt_dest.theta);
  // SupervisionChemin(pt_dest);
}

/* Fonction qui permet de prendre un produit */
void Machine::take( ){
  GripperClientSrv gsrv;
  gsrv.gripper_uppdate(true); 
}

/* Fonction qui permet de deposer un produit */
void Machine::let( ){
  GripperClientSrv gsrv;
  gsrv.gripper_uppdate(false); 
}

void Machine::readlights(){
  ROS_INFO(" Starting exploring the lights ");
  /*std::vector<manager_msg::LightSpec> lSpec;
  lSpec[0].color = 0;
  lSpec[0].state = 0;
  lSpec[1].color = 1;
  lSpec[1].state = 1;
  FeuClientAction f_c;
  f_c.lightsStates(lSpec); */
  ExploInfoSubscriber ei_sub;
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/ExplorationInfo",1000,&ExploInfoSubscriber::tesCallback, &ei_sub);
}

void Machine::startFinalAp(int8_t machineType, int8_t machineSide, int8_t machineParameter){
  FinalApproachingClient fa_c;
  fa_c.starting(machineType,machineSide,machineParameter);
}