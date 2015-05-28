#include "Segment.h"
#include "Point.h"
#include "fa_utils.h"
#include <math.h>
#include <cmath>
#include <vector>
#include <ros/ros.h>

Segment::Segment(){}

Segment::Segment(Point a, Point b,int minR,int maxR)
{
	m_min = a;
	m_max = b;
	m_gradient = 0; 
	m_minRanges = minR;
	m_maxRanges = maxR;
}

Segment::~Segment(){}

bool Segment::nilGradient()
{
	if(m_gradient > 0.95 && m_gradient < 1.05) 
	{
		return true;
	}
	else
	{
    	return false;
    }
}


void Segment::linearRegression(std::vector<Point> tabPoints)
{
	float n = (float)tabPoints.size();
	float sumX = 0, sumY = 0;
	float ecartX = 0, ecartY = 0;				//ecart
	float sumEcartXY = 0.0;						//somme des produits des ecarts sur x et y
	float ecartX2  = 0.0, ecartY2  = 0.0;		//somme des ecarts au carre
	float covXY = 0.0, varX = 0.0, varY = 0.0;	//variances et covariances 
	std::vector<Point>::iterator it;
	for(it=tabPoints.begin();it!=tabPoints.end();it++)
	{
		sumX += it->getX();
		sumY += it->getY();
	}
	//calcul des moyennes
	float moyX = sumX/n;
	float moyY = sumY/n;
	//calcul du coefficient de corrélation
	for(it=tabPoints.begin();it!=tabPoints.end();it++)
	{
		ecartX = it->getX() - moyX;
		ecartY = it->getY() - moyY;
		sumEcartXY += ecartX * ecartY;
		ecartX2  += ecartX * ecartX;
		ecartY2  += ecartY * ecartY;
	}
	covXY = sumEcartXY/n;
	varX = ecartX2 /n;if(varX==0) varX = 1;
	varY = ecartY2 /n;if(varY==0) varY = 1;
	float correl = covXY/sqrt(varX * varY);
	setCorrelation(correl*correl);
	m_gradient = atan2(varY,covXY) ;
	m_gradient = fmod(m_gradient,2*M_PI);
	m_gradient= m_gradient - M_PI_2;
	ROS_INFO("\t\tm_gradient = %f",m_gradient);
}

float Segment::distanceLaserSegment(std::vector<float> ranges)
{
	return (ranges[m_minRanges]+ranges[m_maxRanges])/(float)2;
}


