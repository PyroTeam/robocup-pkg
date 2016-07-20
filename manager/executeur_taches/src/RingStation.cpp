#include <string>

#include "RingStation.h"

/* Constructeur */
RingStation::RingStation(int teamColor)
: Machine(teamColor)
{
	m_name += "RS";
	m_faType = FinalApproachingGoal::RS;
	m_type = "RingStation";
	m_greenRing = 0;
	m_yellowRing = 0;
	m_blueRing = 0;
	m_orangeRing = 0;
}

RingStation::RingStation(int teamColor, int nb)
: RingStation(teamColor)
{
	m_name += std::to_string(nb);
  if (nb == 1)  m_activiType = activity::CS1;
  if (nb == 2)  m_activiType = activity::CS2;
}

/* Destructeur */
RingStation::~RingStation(){}

/* Méthodes */
int RingStation::getGreenRing()
{
	return m_greenRing;
}
int RingStation::getYellowRing()
{
	return m_yellowRing;
}
int RingStation::getBlueRing()
{
	return m_blueRing;
}
int RingStation::getOrangeRing()
{
	return m_orangeRing;
}
void RingStation::majGreen(int nbVert)
{
	m_greenRing = nbVert;
}
void RingStation::majYellow(int nbJaune)
{
	m_yellowRing = nbJaune;
}
void RingStation::majBlue(int nbBleu)
{
	m_blueRing = nbBleu;
}
void RingStation::majOrange(int nbOrange)
{
	m_orangeRing = nbOrange;
}

void RingStation::put_ring(int color)
{
	ROS_INFO("Putting a Ring, color : %d", color);

	goTo(m_entryMachine);

	startFinalAp(FinalApproachingGoal::RS,
                     FinalApproachingGoal::IN,
                     FinalApproachingGoal::CONVEYOR);
	let();

  //TODO: demander à la refbox une base de couleur "color" )

  //TODO: attendre fin de livraison

  //XXX: Retourner qqch pour dire que la tâche est réalisée
}

void RingStation::take_ring() // prendre le produit avec un ring ajouté
{
	goTo(m_exitMachine);
	startFinalAp(FinalApproachingGoal::RS,
               FinalApproachingGoal::OUT,
               FinalApproachingGoal::CONVEYOR);
	grip();

  //XXX: Retourner qqch pour dire que la tâche est réalisée
}

void RingStation::bring_base()
{
  //TODO: vérifier si besoin de demander à la refbox l'ajout d'une base additionnelle
  // Fait automatiquement je pense
  goTo(m_entryMachine);
	startFinalAp(FinalApproachingGoal::RS,
               FinalApproachingGoal::IN,
               FinalApproachingGoal::LANE_RS);
	let();

  //XXX: Retourner qqch pour dire que la tâche est réalisée
}
