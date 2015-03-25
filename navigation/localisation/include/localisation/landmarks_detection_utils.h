#ifndef LANDMARKS_DETECTION_UTILS_H
#define LANDMARKS_DETECTION_UTILS_H

#include "laserScan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"
#include "Machine.h"


double dist(Point a, Droite d);

double dist(Point a, Segment s);

Point ortho(Point a, Droite d);

Point ortho(Point a, Segment s);

Modele ransac(std::list<Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts);

void maj(std::list<Point> &list, Modele m);

//rentrer tous les paramètres de RANSAC dans le prototype de findLines
std::list<Modele> findLines(const std::list<Point> &listOfPoints, int NbPtPertinent, double seuil, int NbPts);

Segment build(const std::list<Point> &points);

std::list<Segment> buildSegment(Modele m, double seuil);

std::list<Segment> buildSegments(std::list<Modele> &listOfModeles);

Machine calculateCoordMachine(Segment s);

void maj(std::list<Segment> &list, Segment s);

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments);

geometry_msgs::Pose2D pointToPose2D(Point point);

#endif