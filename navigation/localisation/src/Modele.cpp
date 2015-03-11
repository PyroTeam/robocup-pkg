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

void Modele::update(){
	m_points.clear();
	//pour tous les itérateurs contenu dans la liste d'index
	for (auto &it : m_index){
		//it est une référence sur un élément de m_index
		//*it est le point recherché dans la liste de points
		m_points.push_back(*it);
	}
}

