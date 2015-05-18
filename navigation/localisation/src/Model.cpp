#include "geometry_msgs/Point.h"
#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "landmarks_detection_utils.h"

#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>

Model::Model() : m_correl(0.0)
{

}

Model::~Model()
{

}

Line Model::getLine() const 
{
	return m_line;
}

double Model::getCorrel() const
{
	return m_correl;
}

const std::list<std::list<geometry_msgs::Point>::iterator> &Model::getIndex() const
{
	return m_index;
}

const std::list<geometry_msgs::Point> &Model::getPoints() const
{
	return m_points;
}

void Model::addPoint(geometry_msgs::Point point)
{
	m_points.push_back(point);
}

void Model::setPoints(std::list<geometry_msgs::Point> listOfPoints)
{
	m_points = listOfPoints;
}

void Model::addIndex(std::list<geometry_msgs::Point>::iterator &it)
{
	m_index.push_back(it);
}

void Model::setLine(Line line)
{
	m_line = line;
}

void Model::linReg()
{
	int          n = m_index.size();
	double    sumX = 0.0, sumY = 0.0;
	double     ecX = 0.0,  ecY = 0.0;					//ecart
	double sumEcXY = 0.0;								//somme des produits des écarts sur x et y
	double    ec2X = 0.0, ec2Y = 0.0;					//somme des écarts au carré
	double   covXY = 0.0, varX = 0.0, varY = 0.0;

	for(auto &it : m_index)
	{
		sumX  += it->x;
		sumY  += it->y;
	}

	//calcul des moyennes
	double moyX = sumX/double(n);
	double moyY = sumY/double(n);

	//calcul du coefficient de corrélation
	for(auto &it : m_index)
	{
		ecX   = it->x - moyX;
		ecY   = it->y - moyY;
		sumEcXY += ecX*ecY;

		ec2X += ecX*ecX;
		ec2Y += ecY*ecY;
	}

	covXY = sumEcXY/double(n);
	varX  = ec2X/double(n);
	varY  = ec2Y/double(n);

	double correl = covXY/sqrt(varX * varY);
	m_correl = correl*correl;

	double slope     = covXY/varX;
	double yIntercept = moyY - slope * moyX;

	geometry_msgs::Pose2D p;
	p.x = moyX;
	p.y = moyY;
	p.theta = atan2(slope,1);

	//mise à jour de la droite
	m_line.set( p,
				slope,
				yIntercept);
}

void Model::build(geometry_msgs::Point a, geometry_msgs::Point b)
{
  	m_line.build(pointToPose2D(a),pointToPose2D(b));
}

void Model::update()
{
	m_points.clear();
	//pour tous les itérateurs contenu dans la liste d'index
	for (auto &it : m_index)
	{
		//it est une référence sur un élément de m_index
		//*it est le point recherché dans la liste de points
		m_points.push_back(*it);
	}
}