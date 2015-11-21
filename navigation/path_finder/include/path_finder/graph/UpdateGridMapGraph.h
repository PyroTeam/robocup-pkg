/**
 * \file 		UpdateGridMapGraph.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-19
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_UPDATEGRIDMAPGRAPH_H_
#define PATH_FINDER_UPDATEGRIDMAPGRAPH_H_

#include <memory>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "UpdateGraph.h"
#include "graph/GridMapGraph.h"

class UpdateGridMapGraph : public UpdateGraph
{
public:
    UpdateGridMapGraph(const std::string &topicName, const std::shared_ptr<Graph> &graph);
    virtual ~UpdateGridMapGraph();
protected:
    void mapCallback(const nav_msgs::OccupancyGrid &grid);
};

#endif /* PATH_FINDER_UPDATEGRIDMAPGRAPH_H_ */
