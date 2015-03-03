#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"
#include "line_detection_utils.h"

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

	double pente     = (n*xyS - xS*yS)/(n*xxS - xS*xS);
	double ordOrigin = (yS - pente*xS)/n;

	//mise à jour de la droite
	m_droite.set( Point(0,ordOrigin),
				  atan2(pente,1));
}

void Modele::build(Point a, Point b){
  	m_droite.build(a,b);
}

void Modele::constructFrom(Modele m){
	m_droite = m.getDroite();
	m_index  = m.getIndex();
	m_correl = m.getCorrel();

	//pour tous les itérateurs contenu dans la liste d'index
	for (std::list<std::list<Point>::iterator>::iterator it = m_index.begin();it != m_index.end();++it){
		//it est l'itérateur sur la liste d'itérateurs (appelé index)
		//*it est l'index correspondant à la position du point recherché dans la liste de points
		//*(*it) est donc le point recherché, c'est celui là qu'on ajoute
		addPoint(*(*it));
	}
}

void Modele::buildSegment(){
	m_segment.setAngle(m_angle);

	double max = std::numeric_limits<double>::max();
	double min = std::numeric_limits<double>::min();

	Point a1(min,0.0);
	Point b1(max,0.0);

	for(std::list<Point>::iterator it = m_points.begin();it != m_points.end();++it){
		if (it->x < a1.x){
			a1 = *it;
		}
		if (it->x > b1.x){
			b1 = *it;
		}
	}

	//calculons maintenant les projetés orthogonaux des deux points extrèmes...
	Point a2 = ortho(a1,m_droite);
	Point b2 = ortho(b1,m_droite);
	//...puis la taille du segment en m
	double size = sqrt((a2.x - b2.x)*(a2.x - b2.x) + (a2.y - b2.y)*(a2.y - b2.y));

	m_segment.setPoints(a2,b2);
	m_segment.setSize(size);
}