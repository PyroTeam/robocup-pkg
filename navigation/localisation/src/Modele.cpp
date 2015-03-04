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
	m_correl = 0.0;
}

Modele::~Modele(){

}

void Modele::linReg(){
	int n = m_index.size();
	double  sumX = 0.0, sumY = 0.0;
	double   ecX = 0.0,  ecY = 0.0;						//ecart
	double sumEcXY = 0.0;								//somme des produits des écarts sur x et y
	double  ec2X = 0.0, ec2Y = 0.0;						//somme des écarts au carré
	double covXY = 0.0, varX = 0.0, varY = 0.0;

	for(auto &it : m_index){
		sumX  += it->getX();
		sumY  += it->getY();
	}

	//calcul des moyennes
	double moyX = sumX/double(n);
	double moyY = sumY/double(n);

	//calcul du coefficient de corrélation
	for(auto &it : m_index){
		ecX   = it->getX() - moyX;
		ecY   = it->getY() - moyY;
		sumEcXY = sumEcXY + ecX * ecY;

		ec2X = ec2X + (ecX * ecX);
		ec2Y = ec2Y + (ecY * ecY);
	}
	covXY = sumEcXY/double(n);
	varX  = ec2X/double(n);
	varY  = ec2Y/double(n);

	double correl = covXY/sqrt(varX * varY);
	m_correl = correl*correl;

	double pente     = covXY/varX;
	double ordOrigin = moyY - pente * moyX;

	//mise à jour de la droite
	m_droite.set( Point(moyX,moyY),
				  atan2(pente,1),
				  pente,
				  ordOrigin);
}

void Modele::build(Point a, Point b){
  	m_droite.build(a,b);
}

void Modele::constructFrom(Modele m){
	m_droite = m.getDroite();
	m_index  = m.getIndex();
	m_correl = m.getCorrel();

	//pour tous les itérateurs contenu dans la liste d'index
	for (auto &it : m_index){
		//it est une référence sur un élément de m_index
		//*it est le point recherché dans la liste de points
		addPoint(*it);
	}
}

void Modele::buildSegment(){
	m_segment.setAngle(m_droite.getAngle());

	double max = std::numeric_limits<double>::max();
	double min = std::numeric_limits<double>::min();

	Point a1(min,0.0);
	Point b1(max,0.0);

	for(auto &it : m_points){
		if (it.getX() < a1.getX()){
			a1 = it;
		}
		if (it.getX() > b1.getX()){
			b1 = it;
		}
	}

	//calculons maintenant les projetés orthogonaux des deux points extrèmes...
	Point a2 = ortho(a1,m_droite);
	Point b2 = ortho(b1,m_droite);
	//...puis la taille du segment en m
	double size = sqrt((a2.getX() - b2.getX())*(a2.getX() - b2.getX()) + (a2.getY() - b2.getY())*(a2.getY() - b2.getY()));

	m_segment.setPoints(a2,b2);
	m_segment.setSize(size);
}