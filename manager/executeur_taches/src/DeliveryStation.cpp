#include <string>

#include "DeliveryStation.h"

/* Constructeur */
DeliveryStation::DeliveryStation(int teamColor)
: Machine(teamColor)
{
	m_name += "DS";
	m_faType = FinalApproachingGoal::DS;
  m_activiType = activity::DS;
	m_type = "DeliveryStation";
}

/* Destructeur */
DeliveryStation::~DeliveryStation(){}

/* Méthodes */
void DeliveryStation::deliver()
{
	ROS_INFO("Delivering the product to the ds ");

	goTo(m_entryMachine);

	startFinalAp(FinalApproachingGoal::DS,
               FinalApproachingGoal::IN,
               FinalApproachingGoal::CONVEYOR);
	let();

  //TODO: demander à la refbox la bonne lane

	//TODO: attendre fin de livraison
 }
