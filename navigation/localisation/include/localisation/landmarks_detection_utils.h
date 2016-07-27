/**
 * \file 			landmarks_detection_utils.h
 * \brief			fonctions de segmentations nécessaires à la détection de machines
 * \author		Danel Thomas
 * \date			2016-07-18
 * \copyright 2016, Association de Robotique de Polytech Lille All rights reserved
*/

#ifndef LANDMARKS_DETECTION_UTILS_H
#define LANDMARKS_DETECTION_UTILS_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"

#include "Segment.h"
#include "Line.h"
#include "Model.h"
#include "Machine.h"
#include "LaserScan.h"

Model ransac(std::list<geometry_msgs::Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts);

void reduceList(std::list<geometry_msgs::Point> &list, Model m);

std::list<Model> findLines(const std::list<geometry_msgs::Point> &listOfPoints, int NbPtPertinent, double seuil, int NbPts, std::list<geometry_msgs::Point> &l);

std::list<Segment> buildSegmentsFromOneModel(Model m, double seuil);

std::list<Segment> buildSegmentsFromModels(std::list<Model> &listOfModels);

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments);

#endif
