#ifndef TASK_H
#define TASK_H

#include <string>
#include <list>

class Task{

public:

	Task(int inti, int parametre,int prod, int deb, int fin, int crea, float rat=1, bool en_traitement=false, 
		 int robot=0, int fin_exec=0);
	
	void setTitle(int inti){m_title=inti;}
	int getTitle(){return m_title;}
	void setParameter(int parametre){m_parameter=parametre;}
	int getParameter(){return m_parameter;}
	void setProduct(int prod){m_product=prod;}
	int getProduct(){return m_product;}
	void setBeginningDelivery(int deb){m_beginningDelivery=deb;}
	int getBeginningDelivery(){return m_beginningDelivery;}
	void setEndDelivery(int fin){m_endDelivery=fin;}
	int getEndDelivery(){return m_endDelivery;}
	void setCreation(int crea){m_creation=crea;}
	int getCreation(){return m_creation;}
	void setRatio(float rat){m_ratio=rat;}
	float getRatio(){return m_ratio;}
	void setInProcess(bool en_traitement){m_inProcess=en_traitement;}
	bool getInProcess(){return m_inProcess;}
	void setRobot(int robot){m_robot=robot;}
	int getRobot(){return m_robot;}
	void setEndCarryingOut(int fin){m_endCarryingOut=fin;}
	int getEndCarryingOut(){return m_endCarryingOut;}  
  
	int pointPerProduct();
	bool inTime(double temps);
  
private:

	int m_title;
	int m_parameter;
	int m_product; 
	int m_beginningDelivery;
	int m_endDelivery;
	int m_creation; //temps de creation restant
	float m_ratio;
	bool m_inProcess;
	int m_robot;
	int m_endCarryingOut;

};

#endif

