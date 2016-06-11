#include <string>

#include "CapStation.h"

/* Constructeur */
CapStation::CapStation()
: Machine()
{
	m_name = "CS";
	m_type = "CapStation";
	m_blackCap = 0;
	m_greyCap = 0;
	m_stockID[0] = 1, m_stockID[1] = 1, m_stockID[2] = 1;
	m_capID[0] = 1, m_capID[1] = 1, m_capID[2] = 1;
}

CapStation::CapStation(int nb)
: CapStation()
{
	m_name += std::to_string(nb);
}

/* Destructeur */
CapStation::~CapStation(){}

/* Fonction Virtuelle */
void CapStation::FonctionVirtuelle(){}

/* Méthodes */
int CapStation::getGreyCap()
{
	return m_greyCap;
}
int CapStation::getBlackCap()
{
	return m_blackCap;
}
int CapStation::getStockage(int i)
{
	return  m_stockID[i];
}

void CapStation::majStockID(int i, int val)
{
	m_stockID[i] = val;
}

void CapStation::majBlack(int nbNoir)
{
	m_blackCap = nbNoir;
}
void CapStation::majGrey(int nbGris)
{
	m_greyCap = nbGris;
}

void CapStation::put_cap(int color, int n_robot, int n_order, int machine)
{
	// A verifier si la cs est dispo
	// si OK : (sinon erreur )

	// TOPIC Générateur de taches : infos sur l'avancement de la tache 
	manager_msg::activity msg;
	msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
	ROS_INFO("Putting a Cap, color : %d", color); 

	goTo(this->m_entryMachine);

	this->startFinalAp(finalApproachingGoal::CS, finalApproachingGoal::IN, finalApproachingGoal::CONVEYOR);
	this->let();

	//Communication_RefBox(je veux un cap de couleur "couleur" )

	msg = msgToGT(n_robot,activity::END,machine,n_order); 
}

void CapStation::take_cap(int color, int n_robot, int n_order, int machine)
{
	// A verifier si la cs est dispo (pas en panne uniquement => cz elle sera entrain de faire un cap "noramlement")
	// si OK : (sinon erreur )

	/* TOPIC Générateur de taches : infos sur l'avancement de la tache */
	manager_msg::activity msg;
	msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
	ROS_INFO("Taking a Cap, color : %d", color);

	//Communication_RefBox(give me the product ) 

	goTo(this->m_exitMachine);

	// while(Communication_RefBox(bs n'a terminé de livrer)) { } ???????

	this->startFinalAp(finalApproachingGoal::CS, finalApproachingGoal::OUT, finalApproachingGoal::CONVEYOR);
	this->take();
	msg = msgToGT(n_robot,activity::END,machine,n_order);
}

void CapStation::stock(int id, int n_robot, int n_order,int machine)
{
	manager_msg::activity msg;
	int8_t place;

	/* TOPIC Générateur de taches : infos sur l'avancement de la tache */
	msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
	ROS_INFO("Stocking @ place : %d", id);

	goTo(this->m_entryMachine);
	
	if(id == 0) place = finalApproachingGoal::S1;
	else if(id == 1) place = finalApproachingGoal::S2; 
	else if(id == 2) place = finalApproachingGoal::S3;

	this->startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::IN,place);
	this->take();
	msg = msgToGT(n_robot,activity::END,machine,n_order);
}

void CapStation::destock(int id, int n_robot, int n_order,int machine)
{
	manager_msg::activity msg;
	int8_t place;

	/* TOPIC Générateur de taches : infos sur l'avancement de la tache */
	msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
	ROS_INFO("Destocking @ place : %d", id);

	goTo(this->m_entryMachine);
	
	if(id == 0) place = finalApproachingGoal::S1;
	else if(id == 1) place = finalApproachingGoal::S2; 
	else if(id == 2) place = finalApproachingGoal::S3;

	this->startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::IN,place);
	this->let();
	msg = msgToGT(n_robot,activity::END,machine,n_order);
}

void CapStation::uncap(int color, int n_robot, int n_order,int machine)
{
	// A verifier si la cs est dispo
	// si OK : (sinon erreur )
 
	/* TOPIC Générateur de taches : infos sur l'avancement de la tache */
	manager_msg::activity msg;
	int8_t place;
	msg = msgToGT(n_robot,activity::IN_PROGRESS,machine,n_order); 
	ROS_INFO("Uncaping");

	goTo(this->m_entryMachine);

	if(m_capID[0] == 1)
	{ 
		place = finalApproachingGoal::S1;
		m_capID[0] == 0;
		m_stockID[0] = 0;
	}
	else if(m_capID[1] == 1) 
	{
		place = finalApproachingGoal::S2;
		m_capID[1] == 0;
		m_stockID[1] = 0;
	} 
	else if(m_capID[2] == 1)
	{ 
		place = finalApproachingGoal::S3;
		m_capID[2] == 0;
		m_stockID[2] = 0;
	}

	this->startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::IN,finalApproachingGoal::CONVEYOR);
	this->take();

	this->startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::IN,finalApproachingGoal::CONVEYOR);
	this->let();

	//Communication_RefBox( Uncap )

	msg = msgToGT(n_robot,activity::END,machine,n_order);

}


