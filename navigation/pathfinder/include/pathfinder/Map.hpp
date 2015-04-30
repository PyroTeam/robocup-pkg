#ifndef _HEADER_MAP_
#define _HEADER_MAP_

#include "Point.hpp"
#include "Objet.hpp"
#include "ros/ros.h"
#include <set>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <cfloat>

#include "nav_msgs/OccupancyGrid.h"

enum typeHeuristic{MANHATTAN,EUCLIDEAN,Chebyshev};

class Map
{
public:
	Map();
	~Map();

	// AStar
	void setAllowDiagonal(bool allowDiagonal);
	void setCrossCorner(bool crossCorner);
	void setPoidsHeuristic(signed int poids);
	float heuristic(Point const& pointDepart, Point const& pointDistant);
	float heuristicManhattan(Point const& pointDepart, Point const& pointDistant);
	float heuristicEuclidean(Point const& pointDepart, Point const& pointDistant);
	float heuristicChebyshev(Point const& pointDepart, Point const& pointDistant);
	void setHeuristicFunction(typeHeuristic heuristicFonction);
	bool isFreeAt(signed int li, signed int col);
	signed int getVoisins(std::vector<Point*> &voisins, Point *oirigin);
	signed int computeAStar(std::vector<Point*> &chemin, Point *startPoint, Point *endPoint);
	signed int getPointAt(signed int ligne, signed int colonne, Point*& point) const;
	signed int getNearestPoint(float x, float y, Point*& point) const;
	void reset();
	bool getClean();
	void setClean(bool c);

	// Grid
	void gridCallback(nav_msgs::OccupancyGridConstPtr grid);
	void constructMap(nav_msgs::OccupancyGridConstPtr grid);
	void constructMap();

private:
	// Machines
	const static int nbProductionMachine = 24;
	const static int nbDeliveryMachine = 6;
	const static int nbRecyclingMachine = 2;
	Objet *_production_machine[nbProductionMachine];
	Objet *_delivery_machine[nbDeliveryMachine];
	Objet *_recycling_machine[nbRecyclingMachine];

	// Points
	const static int nbPointsLignes = 180;
	const static int nbPointsColonnes = 280;
	Point *_pointsPassage[nbPointsLignes][nbPointsColonnes];

	// AStar	
	bool _allowDiagonal;
	bool _crossCorner;
	signed int _poidsHeuristic;
	typeHeuristic _heuristicFonction;
	bool _clean;

	// Grid
	float _resolution;
	float _height;
	float _width;
	float _origin_x;
	float _origin_y;
};

class CompareF
{
public:
    bool operator()(Point* const& a, Point* const& b) const
    {
    	float valFa = a->getF()*1000000+((a->getLigne()*100)%100)+(a->getColonne()%100);
    	float valFb = b->getF()*1000000+((b->getLigne()*100)%100)+(b->getColonne()%100);
        return (valFa < valFb);
    }
};

#endif	// _HEADER_MAP_