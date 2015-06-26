#include "Segment.h"
#include "Point.h"
#include "fa_utils.h"
#include <cmath>
#include <vector>

Segment::Segment(){}

Segment::Segment(Point a, Point b,int min_r,int max_r){
  m_min = a;
  m_max = b;
  m_pente = (b.gety()-a.gety()) / (b.getx()-a.getx()); 
	m_min_ranges = min_r;
	m_max_ranges = max_r;
}

Segment::~Segment(){}

bool Segment::pente_nulle(){
  if(m_pente > 0.95 && m_pente < 1.05) 
    return true;
  else 
    return false;
}


void Segment::regression_lineaire(std::vector<Point> tabpoints){
  float n = (float)tabpoints.size();
	float sum_x = 0, sum_y = 0;
	float ecart_x = 0, ecart_y = 0;						//ecart
	float sum_ecart_xy = 0.0;									//somme des produits des ecarts sur x et y
	float ecart_x2  = 0.0, ecart_y2  = 0.0;		//somme des ecarts au carre
	float covXY = 0.0, varX = 0.0, varY = 0.0;//variances et covariances 
	std::vector<Point>::iterator it;
	
	for(it=tabpoints.begin();it!=tabpoints.end();it++){
		sum_x += it->getx();
		sum_y += it->gety();
	}
	//calcul des moyennes
	float moyX = sum_x/n;
	float moyY = sum_y/n;
	//calcul du coefficient de corrélation
	for(it=tabpoints.begin();it!=tabpoints.end();it++){
		ecart_x = it->getx() - moyX;
		ecart_y = it->gety() - moyY;
		sum_ecart_xy += ecart_x * ecart_y;
		ecart_x2  += ecart_x * ecart_x;
		ecart_y2  += ecart_y * ecart_y;
	}
	covXY = sum_ecart_xy/n;
	varX = ecart_x2 /n;if(varX==0) varX = 1;
	varY = ecart_y2 /n;if(varY==0) varY = 1;
	float correl = covXY/sqrt(varX * varY);
	set_correlation(correl*correl);
	m_angle = atan2(varX,covXY) - M_PI_2;
	
}
