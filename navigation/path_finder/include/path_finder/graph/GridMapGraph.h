/**
 * \file 		GridMapGraph.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef PATH_FINDER_GRIDMAPGRAPH_H_
#define PATH_FINDER_GRIDMAPGRAPH_H_

#include <memory>
#include "Heuristic.h"
#include "Graph.h"
#include <nav_msgs/OccupancyGrid.h>
#include "occupancy_grid_utils/occupancy_grid_utils.h"

/**
 * \class GridMapGraph
 * \brief Classe concrète dérivée de Graph pour les nav_msgs::OccupancyGrid
 *
 * La classe contient une gridMap de type nav_msgs::OccupancyGrid
 *
 */
class GridMapGraph : public Graph
{
public:
    GridMapGraph();
    virtual ~GridMapGraph();

    void setGridMap(const nav_msgs::OccupancyGrid &grid);
    virtual void getSuccessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &succ) override;
    virtual void getPredecessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &pred) override;
    virtual void getClosestNode(const std::shared_ptr<State> &state, std::shared_ptr<State> &closestState) const override;
protected:
    nav_msgs::OccupancyGrid m_grid;
};

#endif /* PATH_FINDER_GRIDMAPGRAPH_H_ */
