#include <string>

#include "CapStation.h"

/* Constructeur */
CapStation::CapStation(){
  m_type = "CapStation";
  m_capNoir = 0;
  m_capGris = 0;
  m_stock[0] = 0; m_stock[1] = 0; m_stock[2] = 0;
  m_stockage.x = 0.0;
  m_stockage.y = 0.0;
  m_stockage.theta = 0.0; 
}

/* Destructeur */
CapStation::~CapStation(){}

/* Fonction Virtuelle */
void CapStation::FonctionVirtuelle(){}

/* Méthodes */
int CapStation::getCapGris(){
  return m_capGris;
}
int CapStation::getCapNoir(){
  return m_capNoir;
}
geometry_msgs::Pose2D CapStation::getStockage(){
  return m_stockage;
}

void CapStation::majNoir(int nbNoir){
  m_capNoir = nbNoir;
}
void CapStation::majGris(int nbGris){
  m_capGris = nbGris;
}

//int CapStation::stocker(){
	/* MANQUE DU CODE */
/*	AllerMachine(m_stockage);
	int i = 1;
	while(i <= m_stock.length && !m_stock[i]){
		i++;
	} 
	return i;
}*/
