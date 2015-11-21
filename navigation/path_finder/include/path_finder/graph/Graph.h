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
#include <list>

#include "Heuristic.h"
#include "search_algo/SearchAlgo.h"
#include "search_algo/State.h"

class Graph
{
public:
    Graph();
    virtual ~Graph();

    virtual void search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState) = 0;
    void setHeuristic(const std::shared_ptr<Heuristic> &heuristic);
    void setSearchAlgo(const std::shared_ptr<SearchAlgo> &searchAlgo);
    virtual void getSuccessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &succ) = 0;
    virtual void getPredecessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &pred) = 0;
    double evaluateHeuristic(State &startState, State &endState);
protected:
    bool m_isInit;
    std::shared_ptr<Heuristic> m_heuristic;
    std::shared_ptr<SearchAlgo> m_searchAlgo;
};

#endif /* PATH_FINDER_GRAPH_H_ */
