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

deplacement_msg::Point convertPtToDeplMsgPt(const Point &point){
	deplacement_msg::Point p;
	p.x = point.getX();
	p.y = point.getY();

	return p;
}

deplacement_msg::Points convertPtsToDeplMsgPts(const std::list<Point> &points){
    deplacement_msg::Points listOfDeplMsgPt;
    listOfDeplMsgPt.points.resize(points.size());

    for (std::list<Point>::const_iterator it = points.begin(); it != points.end(); ++it){
        deplacement_msg::Point pt = convertPtToDeplMsgPt(*it);
        listOfDeplMsgPt.points.push_back(pt);
    }

    return listOfDeplMsgPt;
}

deplacement_msg::Droite convertMdlToDeplMsgDroite(const Modele &modele){
	//on récupère les infos nécessaires du modele
	Droite d     = modele.getDroite();
  Point p      = d.getPoint();
  double angle = d.getAngle();

  deplacement_msg::Droite msgDroite;
  msgDroite.point = convertPtToDeplMsgPt(p);
  msgDroite.angle = angle;

  return msgDroite;
}

deplacement_msg::Droites convertModelesToDeplMsgDroites(const std::list<Modele> &modeles){
    deplacement_msg::Droites droites;
    droites.droites.resize(modeles.size());

    for (std::list<Modele>::const_iterator it = modeles.begin(); it != modeles.end(); ++it){
    	deplacement_msg::Droite d = convertMdlToDeplMsgDroite(*it);
      droites.droites.push_back(d);
    }

    return droites;
}