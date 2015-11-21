/**
 * \file 		aStarSearch.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-19
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_ASTARSEARCH_H_
#define PATH_FINDER_ASTARSEARCH_H_

#include <memory>
#include "searchAlgo.h"

class AStarSearch : public SearchAlgo
{
public:
    AStarSearch(const std::shared_ptr<Graph> &graph);
    virtual ~AStarSearch();
protected:
    //TODO m_openList
    //TODO m_closeList
};

#endif /* PATH_FINDER_ASTARSEARCH_H_ */
