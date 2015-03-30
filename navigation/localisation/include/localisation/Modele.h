#ifndef MODELE_H
#define MODELE_H

#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"

#include <list>

class Modele{
public:
	Modele();
	~Modele();

	Droite getDroite() const {
		return m_droite;
	}
	double getCorrel() const {
		return m_correl;
	}
	const std::list<std::list<Point>::iterator> &getIndex() const{
		return m_index;
	}
	const std::list<Point> &getPoints() const{
		return m_points;
	}

	void addPoint(Point point){
		m_points.push_back(point);
	}
	void setPoints(std::list<Point> listOfPoints){
		m_points = listOfPoints;
	}
	void addIndex(std::list<Point>::iterator &it){
		m_index.push_back(it);
	}
	void setDroite(Droite droite){
		m_droite=droite;
	}
 
	void linReg();
	void build(Point a, Point b);
	void update();

private:
	Droite             					  m_droite;
	std::list<Point>   				      m_points;   //rempli uniquement si le modele est le meilleur_modele
	std::list<std::list<Point>::iterator> m_index;
	double             					  m_correl;
};

#endif