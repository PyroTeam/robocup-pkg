/**
 * \file 		searchAlgo.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_SEARCHALGO_H_
#define PATH_FINDER_SEARCHALGO_H_

#include <memory>

class Graph;

class SearchAlgo
{
public:
    SearchAlgo(const std::shared_ptr<Graph> &graph);
    virtual ~SearchAlgo();

protected:
    std::shared_ptr<Graph> m_graph;
    //TODO State m_currentState;
};

#endif /* PATH_FINDER_SEARCHALGO_H_ */
