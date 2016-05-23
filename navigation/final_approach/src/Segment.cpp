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
	m_gradient = fmod(m_gradient, 2*M_PI);
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


geometry_msgs::Pose2D Segment::linearRegression(std::vector<Point> pointsVector)
{
	float nbPoints = (float)pointsVector.size();
	float sumX = 0, sumY = 0;
	float ecartX = 0, ecartY = 0;				// Ecart
	float sumEcartXY = 0.0;						// Somme des produits des ecarts sur x et y
	float ecartX2  = 0.0, ecartY2  = 0.0;		// Somme des ecarts au carre
	float covXY = 0.0, varX = 0.0, varY = 0.0;	// Variances et covariances
	float correl;
	std::vector<Point>::iterator pointsVector_it;

	// Calcul des sommes
	for(pointsVector_it = pointsVector.begin(); pointsVector_it != pointsVector.end(); pointsVector_it++)
	{
		sumX += pointsVector_it->getX();
		sumY += pointsVector_it->getY();
	}

	// Calcul des moyennes
	float moyX = sumX / nbPoints;
	float moyY = sumY / nbPoints;

	// Calcul du coefficient de corrélation
	for(pointsVector_it = pointsVector.begin(); pointsVector_it != pointsVector.end(); pointsVector_it++)
	{
		ecartX = pointsVector_it->getX() - moyX;
		ecartY = pointsVector_it->getY() - moyY;
		sumEcartXY += ecartX * ecartY;
		ecartX2  += ecartX * ecartX;
		ecartY2  += ecartY * ecartY;
	}
	covXY = sumEcartXY / nbPoints;
	varX = ecartX2 / nbPoints; if(varX == 0) varX = 1;
	varY = ecartY2 / nbPoints; if(varY == 0) varY = 1;

	correl = covXY / sqrt(varX * varY);
	m_correlation = correl * correl;

	m_gradient = atan2(varY, covXY);
	m_gradient = fmod(m_gradient, 2*M_PI);
	m_gradient = m_gradient - M_PI_2;

	geometry_msgs::Pose2D pose;
	pose.x = moyX;
	pose.y = moyY;
	pose.theta = atan2(varY, covXY);

	ROS_DEBUG("linearRegression - result : (x, y - theta) -> (%f, %f - %f)"
						, (float)pose.x, (float)pose.y, (float)pose.theta);

	return pose;
}

float Segment::distanceLaserSegment(std::vector<float> ranges)
{
	ROS_DEBUG("distanceLaserSegment - result : ranges (min, max) -> (%d, %d)", m_minRanges, m_maxRanges);
	return (ranges[ (m_minRanges + m_maxRanges) / 2 ]);
}
