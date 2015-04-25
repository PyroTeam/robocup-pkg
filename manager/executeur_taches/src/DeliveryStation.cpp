#include <string>

#include "DeliveryStation.h"

/* Constructeur */
DeliveryStation::DeliveryStation(){
  m_type = "DeliveryStation";
}

/* Destructeur */
DeliveryStation::~DeliveryStation(){}

/* Fonction Virtuelle */
void DeliveryStation::FonctionVirtuelle(){}

/* Méthodes */
void DeliveryStation::deliverToDS(int n_robot, int n_order){ 
	/* TOPIC Générateur de taches : infos sur l'avancement de la tache */
    manager_msg::activity msg;
    msg = msgToGT(n_robot,activity::IN_PROGRESS,activity::DS,n_order); 

    ROS_INFO("Delivering the product to the ds ");


}

