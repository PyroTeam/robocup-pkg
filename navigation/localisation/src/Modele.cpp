#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>

Modele::Modele(){
	double inf = std::numeric_limits<double>::max();
	setErreur(inf);
}

Modele::~Modele(){

}

void Modele::linReg(){
	int i, n = m_index.size();
	double  xS = 0.0,  yS = 0.0, xyS = 0.0, xxS = 0.0;	//S = Somme
	double  xM = 0.0,  yM = 0.0;												//M = Moyenne
	double  xV = 0.0,  yV = 0.0;												//V = Variance
	double xSC = 0.0, ySC = 0.0;												//SC = Somme des Carrés
	double Num = 0.0, Den = 0.0;												//Numérateur et Denominateur du coeff de corrélation

	//calcul des sommes intermédiaires
	for(std::list<std::list<Point>::iterator>::iterator it = m_index.begin();it != m_index.end();++it){
		xS  += (*it)->x;
		yS  += (*it)->y;
		xyS += ((*it)->x) * ((*it)->y);
		xxS += ((*it)->x) * ((*it)->x);
	}

	//calcul des moyennes
	xM = xS/n;
	yM = yS/n;

	//calcul du coefficient de corrélation
	for(std::list<std::list<Point>::iterator>::iterator it = m_index.begin();it != m_index.end();++it){
		xV   =  (*it)->x - xM;
		yV   =  (*it)->y - yM;
		xSC +=  xV * xV;
		ySC +=  yV * yV;
		Num +=  xV * yV;
	}
	Den  =  sqrt(xSC) + sqrt(ySC);
	m_correl = Num/Den;

	//mise à jour de la droite
	m_droite.set( Point(0,(yS - pente*xS)/n),
								atan2((n*xyS - xS*yS)/(n*xxS - xS*xS),1));
}

void Modele::build(Point a, Point b){
  m_droite.build(a,b);
}