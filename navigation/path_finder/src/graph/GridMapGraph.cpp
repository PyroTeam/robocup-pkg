/**
 * \file 		GridMapGraph.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "graph/GridMapGraph.h"

GridMapGraph::GridMapGraph() : Graph()
{

}

GridMapGraph::~GridMapGraph()
{

}

void GridMapGraph::setGridMap(const nav_msgs::OccupancyGrid &grid)
{
    m_isInit = true;
    m_grid = grid;
}

void GridMapGraph::search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState)
{
    if (m_isInit)
    {
        m_searchAlgo->search(startState, endState);
    }
    else
    {
        ROS_ERROR("Tentative de recherche de trajectoire, alors qu'aucune map n'a été reçue.");
    }
}

void GridMapGraph::getSuccessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &succ)
{

}

void GridMapGraph::getPredecessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &pred)
{

}
