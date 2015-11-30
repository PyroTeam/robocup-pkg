/**
 * \file 		PathPlanner.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-28
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "PathPlanner.h"
#include "graph/Graph.h"

PathPlanner::PathPlanner(const std::shared_ptr<Graph> &graph, std::string name) : m_graph(graph)
{

}

PathPlanner::~PathPlanner()
{

}
