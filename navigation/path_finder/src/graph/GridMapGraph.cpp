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

void GridMapGraph::search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState)
{
    m_searchAlgo->search(startState, endState);
}

void GridMapGraph::getSuccessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &succ)
{

}

void GridMapGraph::getPredecessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &pred)
{

}
