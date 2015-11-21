/**
 * \file 		Graph.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_GRAPH_H_
#define PATH_FINDER_GRAPH_H_

#include <memory>

#include "Heuristic.h"
#include "../search_algo/SearchAlgo.h"

class Graph
{
public:
    Graph();
    virtual ~Graph();

    virtual void search() = 0; //TODO
    void setHeuristic(const std::shared_ptr<Heuristic> &heuristic);
    void setSearchAlgo(const std::shared_ptr<SearchAlgo> &searchAlgo);
    virtual void getSuccessors() = 0;//TODO
    virtual void getPredecessors() = 0;//TODO
    double evaluateHeuristic();
protected:
    bool m_isInit;
    std::shared_ptr<Heuristic> m_heuristic;
    std::shared_ptr<SearchAlgo> m_searchAlgo;
};

#endif /* PATH_FINDER_GRAPH_H_ */
