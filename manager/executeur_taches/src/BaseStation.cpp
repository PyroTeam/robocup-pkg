#include <string>
#include "BaseStation.h"
#include <common_utils/types.h>

using namespace common_utils;

/* Constructeur */
BaseStation::BaseStation(int teamColor)
: Machine(teamColor)
{
	m_name += "BS";
	m_faType = FinalApproachingGoal::BS;
  m_activiType = activity::BS;
	m_type = "BaseStation";
}

/* Destructeur */
BaseStation::~BaseStation(){}

/* Méthodes */
bool BaseStation::take(int color)
{
  goTo(this->m_exitMachine);

	//TODO: demander à la refbox une base de couleur "couleur" )

	//TODO: attendre fin de livraison

  // Accoster machine
	this->startFinalAp(FinalApproachingGoal::BS,
                     FinalApproachingGoal::OUT,
                     FinalApproachingGoal::CONVEYOR);
  // Prendre base
	this->grip();
}
