/**
 * \file 		Graph.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef PATH_FINDER_GRAPH_H_
#define PATH_FINDER_GRAPH_H_

#include <memory>
#include <list>
#include <atomic>

#include "Heuristic.h"
#include "search_algo/SearchAlgo.h"
#include "search_algo/State.h"
#include "search_algo/Path.h"

/**
 * \class Graph
 * \brief Classe abstraite représentant un graph orienté
 *
 * La classe offre les interfaces de bases de manipulation du graph,
 * ces fonctionnalités sont configurables.
 * Par exemple, elle possède une méthode de recherche de chemin. L'algorithme de
 * recherche est attribué à la classe et est modifiable (Dijkstra, Astar...)
 *
 */
class Graph
{
public:
    Graph();
    virtual ~Graph();

    void search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState, Path &path);
    void cancelSearch();
    bool isSearchRunning();
    void setHeuristic(const std::shared_ptr<Heuristic> &heuristic);
    void setSearchAlgo(const std::shared_ptr<SearchAlgo> &searchAlgo);
    virtual void getSuccessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &succ) = 0;
    virtual void getPredecessors(const std::shared_ptr<State> &state, std::list<std::shared_ptr<State>> &pred) = 0;
    double evaluateHeuristic(State &startState, State &endState);

    virtual void getClosestNode(const std::shared_ptr<State> &state, std::shared_ptr<State> &closestState) const;
protected:
    bool m_isInit;
    std::shared_ptr<Heuristic> m_heuristic;
    std::shared_ptr<SearchAlgo> m_searchAlgo;
    std::atomic<bool> m_isSearchRunning;
};

#endif /* PATH_FINDER_GRAPH_H_ */
