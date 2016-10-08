#include <string>

#include "RingStation.h"

/* Constructeur */
RingStation::RingStation()
{
	m_type = "RingStation";
	m_greenRing = 0;
	m_yellowRing = 0;
	m_blueRing = 0;
	m_orangeRing = 0;
}

/* Destructeur */
RingStation::~RingStation(){}

/* Fonction Virtuelle */
void RingStation::FonctionVirtuelle(){}

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

void RingStation::put_ring(int color,int n_robot,int n_order,int machine)
{
	// A verifier si la rs est dispo
	// si OK : (sinon erreur )

	/* TOPIC Générateur de taches : infos sur l'avancement de la tache */
	manager_msg::activity msg;
	msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
	ROS_INFO("Putting a Ring, color : %d", color);

	goTo(this->m_entryMachine);

	this->startFinalAp(FinalApproachingGoal::RS, FinalApproachingGoal::IN, FinalApproachingGoal::CONVEYOR);
	this->let();

	//Communication_RefBox(je veux un ring de couleur "couleur" )

	msg = msgToGT(n_robot,activity::END,machine,n_order); 
}

void RingStation::take_ring(int color,int n_robot,int n_order,int machine)
{
	// A verifier si la rs est dispo (pas en panne uniquement => cz elle sera entrain de faire un ring "noramlement")
	// si OK : (sinon erreur )

	/* TOPIC Générateur de taches : infos sur l'avancement de la tache */
	manager_msg::activity msg;
	msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
	ROS_INFO("Taking a Ring, color : %d", color);

	//Communication_RefBox(give me the product ) 

	goTo(this->m_exitMachine);

	// while(Communication_RefBox(rs n'a terminé de livrer))

	this->startFinalAp(FinalApproachingGoal::RS, FinalApproachingGoal::OUT, FinalApproachingGoal::CONVEYOR);
	this->take();
	msg = msgToGT(n_robot,activity::END,machine,n_order);
}