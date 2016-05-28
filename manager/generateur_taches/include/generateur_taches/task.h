/**
 * \file 			task.h
 * \class			Task
 * \brief			classe représentant la tâche élémentaire que puisse demande le générateur de tâches
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-01
 * \copyright       2016, Association de Robotique de Polytech Lille All rights reserved
 */
 
#ifndef TASK_H
#define TASK_H

#include <string>
#include <list>

class Task{

public:

	Task(int inti, int parametre,int prod, int deb, int fin, int crea, int machineTime=0,
	     float rat=1, bool en_traitement=false, int robot=0, int fin_tache=0);
	
	void setTitle(int inti){m_title=inti;}
	void setParameter(int parametre){m_parameter=parametre;}
	void setComplexity(int prod){m_complexity=prod;}
	void setBeginningDelivery(int deb){m_beginningDelivery=deb;}
	void setEndDelivery(int fin){m_endDelivery=fin;}
	void setCreation(int crea){m_creation=crea;}
	void setRatio(float rat){m_ratio=rat;}
	void setInProcess(bool en_traitement){m_inProcess=en_traitement;}
	void setRobot(int robot){m_robot=robot;}
	void setTaskEnd(int fin){m_taskEnd=fin;}
	void setMachineTime(int m){m_machineTime = m;} 

	int getTitle(){return m_title;}
	int getParameter(){return m_parameter;}
	int getComplexity(){return m_complexity;}
	int getBeginningDelivery(){return m_beginningDelivery;}
	int getEndDelivery(){return m_endDelivery;}
	int getCreation(){return m_creation;}
	float getRatio(){return m_ratio;}
	bool getInProcess(){return m_inProcess;}
	int getRobot(){return m_robot;}
	int getTaskEnd(){return m_taskEnd;}
	int getMachineTime(){return m_machineTime;}
	
	
/**
 *  \brief		Nombre de point par produit
 *  \return		le nombre de point que vaut un produit
 */  
	int pointPerComplexity();
/**
 *	\brief		vérifie si c'est le bon moment pour livrer
 *	\return		true s'il faut livrer sinon false
 */
	bool inTime(double temps);
  
private:

	int m_title;
	int m_parameter;
	int m_complexity; 
	int m_beginningDelivery;
	int m_endDelivery;
	int m_creation; //temps de creation restant
	float m_ratio;
	bool m_inProcess;
	int m_robot;
	int m_machineTime;
	int m_taskEnd;

};

#endif

