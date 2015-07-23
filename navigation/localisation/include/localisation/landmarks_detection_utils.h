#ifndef LANDMARKS_DETECTION_UTILS_H
#define LANDMARKS_DETECTION_UTILS_H

#include "geometry_msgs/Point.h"

class Segment;
class Line;
class Model;
class Machine;
class laserScan;



Model ransac(std::list<geometry_msgs::Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts);

void maj(std::list<geometry_msgs::Point> &list, Model m);

std::list<Model> findLines(const std::list<geometry_msgs::Point> &listOfPoints, int NbPtPertinent, double seuil, int NbPts, std::list<geometry_msgs::Point> &l);

Segment build(const std::list<geometry_msgs::Point> &points);

std::list<Segment> buildSegmentsFromOneModel(Model m, double seuil);

std::list<Segment> buildSegmentsFromModels(std::list<Model> &listOfModels);

Machine calculateCoordMachine(Segment s);

void maj(std::list<Segment> &list, Segment s);

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments);

geometry_msgs::Pose2D pointToPose2D(geometry_msgs::Point point);

geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D pose2d);

#endif