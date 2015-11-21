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

#include <memory>
#include "Heuristic.h"
#include "Graph.h"
#include <nav_msgs/OccupancyGrid.h>
#include "occupancy_grid_utils/occupancy_grid_utils.h"

class GridMapGraph : public Graph
{
public:
    GridMapGraph();
    virtual ~GridMapGraph();

    void setGridMap(const nav_msgs::OccupancyGrid &grid);
    virtual void search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState) override;
    virtual void getSuccessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &succ) override;
    virtual void getPredecessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &pred) override;
protected:
    nav_msgs::OccupancyGrid m_grid;
};

#endif /* PATH_FINDER_GRIDMAPGRAPH_H_ */
