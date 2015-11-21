/**
 * \file 		UpdateGraph.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-19
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_UPDATEGRAPH_H_
#define PATH_FINDER_UPDATEGRAPH_H_

#include <memory>
#include <ros/ros.h>
#include "Graph.h"

class UpdateGraph
{
public:
    UpdateGraph();
    virtual ~UpdateGraph();
protected:
    std::shared_ptr<Graph> m_graph;
    ros::Subscriber m_graph_sub;
    bool m_planningPending;
};

#endif /* PATH_FINDER_UPDATEGRAPH_H_ */
