#include "docking/Segment.h"

#include "docking/Point.h"
#include "docking/fa_utils.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

#include <math.h>
#include <cmath>
#include <vector>

Segment::Segment()
	: m_nullSegment(true)
{
}

Segment::Segment(geometry_msgs::Point min, geometry_msgs::Point max, float gradient)
	: m_nullSegment(true)
{
	m_minPoint = min;
	m_maxPoint = max;
	m_gradient = fmod(m_gradient, 2*M_PI);
	m_gradient= m_gradient - M_PI_2;
}

Segment::Segment(Point a, Point b, int minR, int maxR)
	: m_nullSegment(true)
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
	assert(!pointsVector.empty());

	float nbPoints = (float)pointsVector.size();
	float sumX = 0, sumY = 0;
	float ecartX = 0, ecartY = 0;									// Ecart
	float sumEcartXY = 0.0;											// Somme des produits des ecarts sur x et y
	float sumEcartX2  = 0.0, sumEcartY2  = 0.0;						// Somme des ecarts au carre
	float covarianceXY = 0.0, varianceX = 0.0, varianceY = 0.0;		// Variances et covariances
	float slope = 0.0;												// Pente de la droite
	float origin = 0.0;												// Ordonnée à l'origine
	float X0 = 0.0;													// Abscisse de la droite pour Y = 0
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

	// Enregistrement du point milieu (centre de gravité du segment)
	m_middlePoint.x = moyX;
	m_middlePoint.y = moyY;
	m_middlePoint.z = 0.0;

	// Calcul de la variance et covariance pour en déduire l'équation de la droite (par la méthode des moindre carrés)
	for(pointsVector_it = pointsVector.begin(); pointsVector_it != pointsVector.end(); pointsVector_it++)
	{
		ecartX = pointsVector_it->getX() - moyX;
		ecartY = pointsVector_it->getY() - moyY;

		sumEcartXY 	+= ecartX * ecartY;
		sumEcartX2  += ecartX * ecartX;
		sumEcartY2  += ecartY * ecartY;
	}
	covarianceXY = sumEcartXY / nbPoints;
	varianceX = sumEcartX2 / nbPoints; if(varianceX == 0) varianceX = 1;
	varianceY = sumEcartY2 / nbPoints; if(varianceY == 0) varianceY = 1;

	// Sauvegarde l'angle de la droite (en radian)
	m_angle = atan2(varianceY, covarianceXY);
	m_angle = fmod(m_angle, 2*M_PI);

	slope = tan(m_angle);
	origin = moyY - slope * moyX;
	X0 = (slope != 0)? -origin/slope : 0;  // Si la pente est nul, on considère que (0,0) sera le point otho

	// Enregistrement du point orthogonal
	m_orthoPoint.x = X0;
	m_orthoPoint.y = 0.0;
	m_orthoPoint.z = 0.0;

	// Marquer le segment comme construit
	m_nullSegment = false;



	// XXX: A partir d'ici je ne garantie pas le code. D'ailleurs rien d'ici ne devrait m'être utile.
	// A relire, documenter ou retirer
	correl = covarianceXY / sqrt(varianceX * varianceY);
	ROS_DEBUG_NAMED("segmentConstruction","Coeff correlation: %4.2f | Droite (%5.3f, %5.3f) Angle: %5.3f | Pente: %4.2f", correl, moyX, moyY, m_angle, slope);
	m_correlation = correl * correl;

	m_gradient = atan2(varianceY, covarianceXY);
	m_gradient = fmod(m_gradient, 2*M_PI);
	// m_angle = m_gradient; // TODO: Remove
	m_gradient = m_gradient - M_PI_2;

	geometry_msgs::Pose2D pose;
	pose.x = moyX;
	pose.y = moyY;
	pose.theta = atan2(varianceY, covarianceXY);

	ROS_DEBUG_NAMED("segmentConstruction","linearRegression - result : (x, y - theta) -> (%f, %f - %f)"
						, (float)pose.x, (float)pose.y, (float)pose.theta);

	return pose;
}

float Segment::distanceLaserSegment()
{
	if (m_nullSegment)
	{
		ROS_ERROR("Requested a distance on a not yet constructed segment");
		return -1;
	}

	return sqrt(m_middlePoint.x*m_middlePoint.x + m_middlePoint.y*m_middlePoint.y);
}

float Segment::distanceOrthoLaserSegment()
{
	if (m_nullSegment)
	{
		ROS_ERROR("Requested a distance on a not yet constructed segment");
		return -1;
	}

	// Le point orthogonal étant déjà trouvé, c'est aussi simple que ça
	return m_orthoPoint.x;
}
