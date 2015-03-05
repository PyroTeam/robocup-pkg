#include "laserScan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <list>
#include <algorithm>

#include "line_detection_utils.h"

std::list<Segment> buildSegment(Modele m, double seuil){
	std::list<Segment> listOfSegments;
	std::list<Point> tmp;
	std::list<Point>::iterator previousPoint = m.getPoints().begin();

	for(std::list<Point>::iterator it : m.getPoints()){
		double d = sqrt((it->getY()-previousPoint->getY())*(it->getY()-previousPoint->getY()) +
                        (it->getX()-previousPoint->getX())*(it->getX()-previousPoint->getX()));
		if (d < seuil){
			tmp.push_back(*it);
		}
		else {
			//construction du segment
			Segment s;
			s.setAngle(m.getDroite().getAngle());
			s.build(tmp);
			listOfSegments.push_back(s);
			tmp.clear();
		}
		previousPoint = it;
	}
	if (tmp.size() >= 2){
		Segment s;
		s.build(tmp);
		listOfSegments.push_back(s);
	}

	return listOfSegments;
}

std::list<Segment> buildSegments(std::list<Modele> &listOfModeles){
	std::list<Segment> listOfSegments;
	//pour tous les modèles de la liste
	for (auto &it : listOfModeles){
		std::list<Segment> listTmp = buildSegment(it, 0.35);
		listOfSegments.merge(listTmp);
	}

	return listOfSegments;
}