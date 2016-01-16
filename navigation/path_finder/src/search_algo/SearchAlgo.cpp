/**
 * \file 		SearchAlgo.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "search_algo/SearchAlgo.h"
#include "graph/Graph.h"

SearchAlgo::SearchAlgo(const std::shared_ptr<Graph> &graph, bool reverse):m_graph(graph), m_reverse(reverse),  m_cancelSearch(false)
{

}

SearchAlgo::~SearchAlgo()
{

}

/**
 * Méthode d'annulation d'une recherche, utile si la recherche est lancée dans un thread
 *
 */
void SearchAlgo::cancelSearch()
{
     m_cancelSearch = true;
}
