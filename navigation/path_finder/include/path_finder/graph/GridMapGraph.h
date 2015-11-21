/**
 * \file 		GridMapGraph.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_GRIDMAPGRAPH_H_
#define PATH_FINDER_GRIDMAPGRAPH_H_

#include "Heuristic.h"
#include "Graph.h"
#include <nav_msgs/OccupancyGrid.h>

class GridMapGraph : public Graph
{
public:
    GridMapGraph();
    virtual ~GridMapGraph();

    virtual void search() override; //TODO
    virtual void getSuccessors() override;//TODO
    virtual void getPredecessors() override;//TODO
};

#endif /* PATH_FINDER_GRIDMAPGRAPH_H_ */
