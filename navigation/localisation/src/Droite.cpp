#include "Point.h"
#include "Droite.h"
#include <cmath>
#include <limits>

Droite::Droite(){
	double inf = std::numeric_limits<double>::max();
	setAngle(0.0);
	setErreur(inf);
}

Droite::~Droite(){

}

double Droite::linReg(std::vector<Point> points){
	int i;
	double  xS = 0.0,  yS = 0.0, xyS = 0.0, xxS = 0.0;		//S = somme
	double  xM = 0.0,  yM = 0.0;							//M = moyenne
	double  xV = 0.0,  yV = 0.0;							//SC = somme des carrés
	double xSC = 0.0, ySC = 0.0;							//V = variance
	double Num = 0.0, Den = 0.0;							//Num et den du coeff de corrélation
	double err = 0.0;										//coeff de corrélation	

	//var utiles pour le calcul des paramètres de la droite
	for (i = 0; i < points.size(); i++){
		xS  += points[i].x;
		yS  += points[i].y;
		xyS += points[i].x * points[i].y;
		xxS += points[i].x * points[i].x;
	}

	xM   =  xS/points.size();
	yM   =  yS/points.size();

	//calcul du coefficient de corrélation
	for (i = 0; i < points.size(); i++){
		//pour calculer le coeff de corrélation
		xV   =  points[i].x - xM;
		yV   =  points[i].y - yM;
		xSC +=  xV * xV;
		ySC +=  yV * yV;
		Num +=  xV * yV;
	}
	Den  =  sqrt(xSC) + sqrt(ySC);
	err = Num/Den;

	//mise à jour de la droite
	m_a = (points.size()*xyS - xS*yS)/(points.size()*xxS - xS*xS);
	m_b = (yS - m_a*xS)/points.size();
	m_angle = atan2(m_a,1);
	m_point = Point(0,m_b);

	return err;
}