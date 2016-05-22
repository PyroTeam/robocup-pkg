#include "final_approach/Segment.h"

#include "final_approach/Point.h"
#include "final_approach/fa_utils.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

#include <math.h>
#include <cmath>
#include <vector>

Segment::Segment(){}

Segment::Segment(geometry_msgs::Point min, geometry_msgs::Point max, float gradient)
{
	m_minPoint = min;
	m_maxPoint = max;
	m_gradient = fmod(m_gradient,2*M_PI);
	m_gradient= m_gradient - M_PI_2;
}

Segment::Segment(Point a, Point b, int minR, int maxR)
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


geometry_msgs::Pose2D Segment::linearRegression(std::vector<Point> tabPoints)
{
	float n = (float)tabPoints.size();
	float sumX = 0, sumY = 0;
	float ecartX = 0, ecartY = 0;				// Ecart
	float sumEcartXY = 0.0;						// Somme des produits des ecarts sur x et y
	float ecartX2  = 0.0, ecartY2  = 0.0;		// Somme des ecarts au carre
	float covXY = 0.0, varX = 0.0, varY = 0.0;	// Variances et covariances
	std::vector<Point>::iterator it;

	for(it=tabPoints.begin();it!=tabPoints.end();it++)
	{
		sumX += it->getX();
		sumY += it->getY();
	}

	// Calcul des moyennes
	float moyX = sumX/n;
	float moyY = sumY/n;

	// Calcul du coefficient de corrélation
	for(it=tabPoints.begin();it!=tabPoints.end();it++)
	{
		ecartX = it->getX() - moyX;
		ecartY = it->getY() - moyY;
		sumEcartXY += ecartX * ecartY;
		ecartX2  += ecartX * ecartX;
		ecartY2  += ecartY * ecartY;
	}

	covXY = sumEcartXY / n;
	varX = ecartX2 / n; if(varX == 0) varX = 1;
	varY = ecartY2 / n; if(varY == 0) varY = 1;

	float correl = covXY / sqrt(varX * varY);
	setCorrelation(correl * correl);

	m_gradient = atan2(varY, covXY);
	m_gradient = fmod(m_gradient, 2*M_PI);
	m_gradient = m_gradient - M_PI_2;

	geometry_msgs::Pose2D p;
	p.x = moyX;
	p.y = moyY;
	p.theta = atan2(varY, covXY);

	ROS_DEBUG("linearRegression - result : (x, y - theta) -> (%f, %f - %f)", (float)p.x, (float)p.y, (float)p.theta);
	return p;
}

float Segment::distanceLaserSegment(std::vector<float> ranges)
{
	ROS_DEBUG("distanceLaserSegment - result : ranges (min, max) -> (%d, %d)", m_minRanges, m_maxRanges);
	return (ranges[ (m_minRanges + m_maxRanges) / 2 ]);
}
