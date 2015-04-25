#include "laserScan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"
#include "Machine.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <list>
#include <algorithm>
#include <iterator>
#include <limits>

#include "line_detection_utils.h"

double dist(Point a, Segment s){
	Point b = s.getMin();
  	Point u(cos(s.getAngle()), sin(s.getAngle()));
  	Point ba(a.getX()-b.getX(), a.getY()-b.getY());

	return std::abs(u.getX()*ba.getY() - ba.getX()*u.getY()) / sqrt(u.getX()*u.getX() + u.getY()*u.getY());
}

Point ortho(Point a, Segment s){
  double distance = dist(a,s);
  double dx = distance*cos(s.getAngle());
  double dy = distance*sin(s.getAngle());
  Point p(a.getX() - dx, a.getY() + dy);

  return p;
}

Segment build(const std::list<Point> &points){
	Segment s;
	Point a;
	Point b;

	//on calcule les coordonnées des projetés orthogonaux des deux points extrêmes
  	double min = std::numeric_limits<double>::max();
  	double max = std::numeric_limits<double>::min();

  	for (auto &it : points){
  		if (it.getX() < min){
  			min = it.getX();
  			a = it;
  		}
  		if (it.getX() > max){
  			max = it.getX();
  			b = it;
  		}
  	}

	//...puis la taille du segment en mètre
	double size = sqrt( (a.getX()-b.getX()) * (a.getX()-b.getX()) +
						(a.getY()-b.getY()) * (a.getY()-b.getY()));

	s.setPoints(a,b);
	s.setSize(size);

  return s;
}

std::list<Segment> buildSegment(Modele m, double seuil){
	std::list<Segment> listOfSegments;
	std::list<Point> tmp;
	std::list<Point>::const_iterator previousPoint = m.getPoints().cbegin();

	//pour chaque points dans la liste de points du modèle
	for(std::list<Point>::const_iterator it = m.getPoints().cbegin(); it != m.getPoints().cend(); ++it){
		//on calcule la distance entre voisins
		double d = sqrt((it->getY()-previousPoint->getY())*(it->getY()-previousPoint->getY()) +
                        (it->getX()-previousPoint->getX())*(it->getX()-previousPoint->getX()));
		
		//si les points sont proches
		if (d < seuil){
			//on sauvegarde ces points dans une liste
			tmp.push_back(*it);
		}
		//sinon (on détecte un seuil important)
		else {
			//on construit un segment à partir de la liste des points qui sont proches
			Segment s = build(tmp);

			//on recalcule l'angle à partir des points extrêmes
			double pente =  (s.getMax().getY()-s.getMin().getY()) /
							(s.getMax().getX()-s.getMin().getX());
			double angle = atan2(pente,1);
			s.setAngle(angle);

			//on enregistre le segment dans la liste de segments
			listOfSegments.push_back(s);
			tmp.clear();
			tmp.push_back(*it);
		}
		//on recommence en partant du point suivant le dernier point
		//qui était dans la liste précédente
		previousPoint = it;
	}
	//pour le dernier point, le seuil ne pouvant plus être dépassé,
	//on construit le dernier segment
	if (tmp.size() >= 2){
		Segment s = build(tmp);
		//on recalcule l'angle à partir des points extrêmes
		double pente =  (s.getMax().getY()-s.getMin().getY()) /
						(s.getMax().getX()-s.getMin().getX());
		double angle = atan2(pente,1);
		s.setAngle(angle);
		listOfSegments.push_back(s);
	}
	tmp.clear();

	return listOfSegments;
}

std::list<Segment> buildSegments(std::list<Modele> &listOfModeles){
	std::list<Segment> listOfSegments;
	//pour tous les modèles de la liste
	for (auto &it : listOfModeles){
		std::list<Segment> listTmp = buildSegment(it, 1);
		//on concatène les listes de segments trouvés à partir de chaque modèle ensemble
		listOfSegments.splice(listOfSegments.end(),listTmp);
	}

	return listOfSegments;
}

std::list<Machine> recognizeMachinesFrom(std::list<Segment> listOfSegments){
	std::list<Machine> tmp;
	int cpt = 1;

	for (auto &it : listOfSegments){
		double angle = it.getAngle();
		double size  = it.getSize();

		double 	gM = 0.70 + 0.05,
				gm = 0.70 - 0.05,
				pM = 0.35 + 0.05,
				pm = 0.35 - 0.05;

		double 	abscisse 	= 0.0,
				ordonnee 	= 0.0,
				orientation = 0.0;

		double absMilieu = (it.getMax().getX() + it.getMin().getX())/2;
		double ordMilieu = (it.getMax().getY() + it.getMin().getY())/2;

		//si c'est un grand côté
		if ((size > gm) && (size < gM)){
			abscisse = absMilieu + 0.35*sin(M_PI_2-angle);
			ordonnee = ordMilieu - 0.35*cos(M_PI_2-angle);
			orientation = angle;
		}
		//si c'est un petit côté
		if ((size > pm) && (size < pM)){
			abscisse = absMilieu + 0.175*sin(M_PI_2-angle);
			ordonnee = ordMilieu - 0.175*cos(M_PI_2-angle);
			orientation = M_PI_2 + angle;
		}

		if (abscisse != 0.0 && ordonnee != 0.0 && orientation != 0.0){
			Machine m(abscisse,ordonnee, orientation);
			tmp.push_back(m);
			m.clear();
		}
		cpt++;
	}

	return tmp;
}