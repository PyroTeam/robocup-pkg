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

// #include "deplacement_msg/Points.h"
// #include "deplacement_msg/Droites.h"

double dist(Point a, Droite d);

Point ortho(Point a, Droite d);

Modele ransac(std::list<Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts);

std::list<Modele> findLines(const std::list<Point> &listOfPoints);

#endif