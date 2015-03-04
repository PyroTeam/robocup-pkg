#ifndef LINE_DETECTION_UTILS_H
#define LINE_DETECTION_UTILS_H

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

#include "deplacement_msg/Points.h"
#include "deplacement_msg/Droites.h"

deplacement_msg::Point convertPtToDeplMsgPt(const Point &point);

deplacement_msg::Points convertPtsToDeplMsgPts(const std::list<Point> &points);

deplacement_msg::Droite convertMdlToDeplMsgDroite(const Modele &modele);

deplacement_msg::Droites convertModelesToDeplMsgDroites(const std::list<Modele> &modeles);

double dist(Point a, Droite d);

Point ortho(Point a, Droite d);

Modele ransac(std::list<Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts);

std::list<Modele> findLines(std::list<Point> &listOfPoints);

#endif