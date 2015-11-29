#ifndef _PATHFINDER__ASTAR_H_
#define _PATHFINDER__ASTAR_H_

#include <set>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <cfloat>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "Point.hpp"

enum typeHeuristic{MANHATTAN,EUCLIDEAN,CHEBYSHEV};

class AStar
{
public:
    AStar();
    ~AStar();

    // AStar
    void setAllowDiagonal(bool allowDiagonal);
    void setCrossCorner(bool crossCorner);
    void setPoidsHeuristic(signed int poids);
    float heuristic(Point const *pointDepart, Point const *pointDistant);
    float heuristicManhattan(Point const *pointDepart, Point const *pointDistant);
    float heuristicEuclidean(Point const *pointDepart, Point const *pointDistant);
    float heuristicChebyshev(Point const *pointDepart, Point const *pointDistant);
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
    void updateMap(nav_msgs::OccupancyGridConstPtr grid);
    void constructMap();
    void destructMap();

private:
    // Points
    const static int nbPointsLignes = 180;
    const static int nbPointsColonnes = 280;
    Point *m_pointsPassage[nbPointsLignes][nbPointsColonnes];

    // AStar    
    bool m_allowDiagonal;
    bool m_crossCorner;
    signed int m_poidsHeuristic;
    typeHeuristic m_heuristicFonction;
    bool m_clean;

    // Grid
    float m_resolution;
    float m_height;
    float m_width;
    float m_origin_x;
    float m_origin_y;
};

class CompareF
{
public:
    bool operator()(Point* const& a, Point* const& b) const
    {
        float valFa = a->getF()*1000000+((a->getRaw()*100)%100)+(a->getColumn()%100);
        float valFb = b->getF()*1000000+((b->getRaw()*100)%100)+(b->getColumn()%100);
        return (valFa < valFb);
    }
};

#endif  // _PATHFINDER__ASTAR_H_