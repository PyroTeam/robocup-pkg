/**
 * \file 		graph.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "graph/graph.h"

Graph::Graph():m_isInit(false), m_heuristic(nullptr), m_searchAlgo(nullptr)
{

}

Graph::~Graph()
{

}

void Graph::setHeuristic(const std::shared_ptr<Heuristic> &heuristic)
{
    m_heuristic = heuristic;
}

void Graph::setSearchAlgo(const std::shared_ptr<SearchAlgo> &searchAlgo)
{
    m_searchAlgo = searchAlgo;
}

double Graph::evaluateHeuristic()
{
    return m_heuristic->evaluate();
}
