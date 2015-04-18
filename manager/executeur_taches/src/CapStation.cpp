#include <string>

#include "CapStation.h"

/* Constructeur */
CapStation::CapStation(){
  m_type = "CapStation";
  m_blackCap = 0;
  m_greyCap = 0;
  m_stockID[0] = 0, m_stockID[1] = 0, m_stockID[2] = 0;
  m_stockage.x = 0.0;
  m_stockage.y = 0.0;
  m_stockage.theta = 0.0; 
}

/* Destructeur */
CapStation::~CapStation(){}

/* Fonction Virtuelle */
void CapStation::FonctionVirtuelle(){}

/* Méthodes */
int CapStation::getGreyCap(){
  return m_greyCap;
}
int CapStation::getBlackCap(){
  return m_blackCap;
}
int CapStation::getStockage(int i){
  return  m_stockID[i];
}

void CapStation::majStockID(int i, int val){
  m_stockID[i] = val;
}
geometry_msgs::Pose2D CapStation::getStockage(){
  return m_stockage;
}

void CapStation::majBlack(int nbNoir){
  m_blackCap = nbNoir;
}
void CapStation::majGrey(int nbGris){
  m_greyCap = nbGris;
}

void CapStation::put_cap(int color, int n_robot, int n_order, int machine){
    /* TOPIC Générateur de taches : infos sur l'avancement de la tache */
    manager_msg::activity msg;
    msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
    ROS_INFO("Putting a Cap, color : %d", color);
}

void CapStation::take_cap(int color, int n_robot, int n_order, int machine){
    /* TOPIC Générateur de taches : infos sur l'avancement de la tache */
    manager_msg::activity msg;
    msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
    ROS_INFO("Taking a Cap, color : %d", color);
}



void CapStation::stock(int id, int n_robot, int n_order,int machine){
    /* TOPIC Générateur de taches : infos sur l'avancement de la tache */
    manager_msg::activity msg;
    msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
    ROS_INFO("Stocking @ place : %d", id);
}

void CapStation::destock(int id, int n_robot, int n_order,int machine){
    /* TOPIC Générateur de taches : infos sur l'avancement de la tache */
    manager_msg::activity msg;
    msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
    ROS_INFO("Destocking @ place : %d", id);
}

void CapStation::uncap(int id, int n_robot, int n_order,int machine){
    /* TOPIC Générateur de taches : infos sur l'avancement de la tache */
    manager_msg::activity msg;
    msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
    ROS_INFO("Uncaping");
}


