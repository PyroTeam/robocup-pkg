#include "final_approach/Segment.h"

#include "final_approach/Point.h"
#include "final_approach/fa_utils.h"

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
}

Segment::Segment(Point a, Point b, int minR, int maxR)
	: m_nullSegment(true)
{
	m_min = a;
	m_max = b;
	m_minRanges = minR;
	m_maxRanges = maxR;
}

Segment::~Segment(){}

geometry_msgs::Pose2D Segment::linearRegression(std::vector<Point> pointsVector)
{
  // pour maintenir la fonctionnalité sans mettre le type Point (r, Phi) dans les common
  std::vector<geometry_msgs::Point> tmp;
  for (auto &it : pointsVector)
  {
    geometry_msgs::Point ptTemp;
    ptTemp.x = it.getX();
    ptTemp.y = it.getY();
    tmp.push_back(ptTemp);
  }

	assert(!pointsVector.empty());

	float slope = 0.0;												// Pente de la droite
	float origin = 0.0;												// Ordonnée à l'origine
	float X0 = 0.0;													// Abscisse de la droite pour Y = 0

  double correl = geometry_utils::linearRegression(tmp, m_middlePoint, m_angle);

	slope = tan(m_angle);
	origin = m_middlePoint.y - slope * m_middlePoint.x;
	X0 = (slope != 0)? -origin/slope : 0;  // Si la pente est nul, on considère que (0,0) sera le point otho

	// Enregistrement du point orthogonal
	m_orthoPoint.x = X0;
	m_orthoPoint.y = 0.0;
	m_orthoPoint.z = 0.0;

	// Marquer le segment comme construit
	m_nullSegment = false;



	// XXX: A partir d'ici je ne garantie pas le code. D'ailleurs rien d'ici ne devrait m'être utile.
	ROS_DEBUG_NAMED("segmentConstruction",
                  "Coeff correlation: %4.2f | Droite (%5.3f, %5.3f) Angle: %5.3f | Pente: %4.2f",
                  correl, m_middlePoint.x, m_middlePoint.y, m_angle, slope);

	m_correlation = correl * correl;

	geometry_msgs::Pose2D pose;
	pose.x = m_middlePoint.x;
	pose.y = m_middlePoint.y;
	pose.theta = m_angle;

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
