#include "geometry_msgs/Point.h"
#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "landmarks_detection_utils.h"
#include "math_functions.h"

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
	std::list<geometry_msgs::Point> pts;
	geometry_msgs::Point p;
	for (auto &it : m_index)
	{
		p = *it;
		pts.push_back(p);
	}
	geometry_msgs::Pose2D pt;
	m_correl = ::linReg(pts, pt);
	m_line.set(pt);
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