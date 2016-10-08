/**
 * \file 		SearchAlgo.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef PATH_FINDER_SEARCHALGO_H_
#define PATH_FINDER_SEARCHALGO_H_

#include <memory>
#include <list>
#include <atomic>
#include "State.h"
#include "Path.h"

class Graph;

/**
 * \class SearchAlgo
 * \brief Classe abstraite pour les algo de recherche sur des Graph
 *
 * La classe propose les interfaces génériques des algortihmes de recherche
 * sur les Graph
 *
 */
class SearchAlgo
{
public:
    SearchAlgo(const std::shared_ptr<Graph> &graph, bool reverse = false);
    virtual ~SearchAlgo();

    virtual void search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState, Path &path) = 0;
    virtual void cancelSearch();
protected:
    std::shared_ptr<Graph> m_graph;
    bool m_reverse;
    std::atomic<bool> m_cancelSearch;
};

#endif /* PATH_FINDER_SEARCHALGO_H_ */
