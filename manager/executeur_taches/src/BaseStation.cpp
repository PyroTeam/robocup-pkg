#include <string>
#include "BaseStation.h"

/* Constructeur */
BaseStation::BaseStation()
: Machine()
{
	m_name = "BS";
	m_faType = finalApproachingGoal::BS;
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
int BaseStation::getRedBase()
{
	return m_redBase;
}
int BaseStation::getBlackBase()
{
	return m_blackBase;
}
int BaseStation::getSilverBase()
{
	return m_silverBase;
}
void BaseStation::majRed(int nbRouge)
{
	m_redBase = nbRouge;
}
void BaseStation::majBlack(int nbNoir)
{
	m_blackBase = nbNoir;
}
void BaseStation::majSilver(int nbArgent)
{
	m_silverBase = nbArgent;
}

void BaseStation::take_base(int color,int n_robot,int n_order)
{
	// A verifier si la bs est dispo
	// si OK : (sinon erreur )

	// TOPIC Générateur de taches : infos sur l'avancement de la tache 
	manager_msg::activity msg;
	msg = msgToGT(n_robot,activity::IN_PROGRESS,activity::BS,n_order); 
	ROS_INFO("Taking a Base, color : %d", color); 

	goTo(this->m_exitMachine);

	//Communication_RefBox(je veux une base de couleur "couleur" )

	// while(Communication_RefBox(bs n'a terminé de livrer)) { }

	this->startFinalAp(finalApproachingGoal::BS, finalApproachingGoal::OUT, finalApproachingGoal::CONVEYOR);
	this->take();
	msg = msgToGT(n_robot,activity::END,activity::BS,n_order); 
}

void BaseStation::bring_base_rs(int color,int n_robot,int n_order,int machine)
{
	manager_msg::activity msg;
	/* 1ere partie : prendre la base */

	this->take_base(color,n_robot,n_order);

	/* 2eme partie : emener la base */

	// A verifier si la bs est dispo
	// si OK : (sinon erreur )
	ROS_INFO("Bringing a Base to a RS"); 

	goTo(this->m_centerMachine);
	this->startFinalAp(finalApproachingGoal::RS, finalApproachingGoal::IN, finalApproachingGoal::LANE_RS);
	this->let();
	msg = msgToGT(n_robot,activity::END,activity::BS,n_order);
}