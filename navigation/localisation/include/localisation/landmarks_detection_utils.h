#ifndef LANDMARKS_DETECTION_UTILS_H
#define LANDMARKS_DETECTION_UTILS_H

#include "laserScan.h"
#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "Machine.h"


double dist(geometry_msgs::Point a, Line d);

double dist(geometry_msgs::Point a, Segment s);

geometry_msgs::Point ortho(geometry_msgs::Point a, Line d);

geometry_msgs::Point ortho(geometry_msgs::Point a, Segment s);

Model ransac(std::list<geometry_msgs::Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts);

void maj(std::list<geometry_msgs::Point> &list, Model m);

std::list<Model> findLines(const std::list<geometry_msgs::Point> &listOfPoints, int NbPtPertinent, double seuil, int NbPts, std::list<geometry_msgs::Point> &l);

double linReg(const std::list<geometry_msgs::Point> &points, geometry_msgs::Pose2D &p);

Segment build(const std::list<geometry_msgs::Point> &points);

std::list<Segment> buildSegmentsFromOneModel(Model m, double seuil);

std::list<Segment> buildSegmentsFromModels(std::list<Model> &listOfModels);

Machine calculateCoordMachine(Segment s);

void maj(std::list<Segment> &list, Segment s);

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments);

geometry_msgs::Pose2D pointToPose2D(geometry_msgs::Point point);

geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D pose2d);

#endif