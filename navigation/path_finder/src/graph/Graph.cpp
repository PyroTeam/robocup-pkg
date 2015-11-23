/**
 * \file 		Graph.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "ros/ros.h"
#include "graph/Graph.h"

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

double Graph::evaluateHeuristic(State &startState, State &endState)
{
    return m_heuristic->evaluate(startState, endState);
}

void Graph::search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState, std::list<std::shared_ptr<State>> &path)
{
    if (m_isInit)
    {
        m_searchAlgo->search(startState, endState, path);
    }
    else
    {
        ROS_ERROR("Tentative de recherche de trajectoire, alors qu'aucune map n'a été reçue.");
    }
}
