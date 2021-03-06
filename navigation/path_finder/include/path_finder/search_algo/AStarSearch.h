/**
 * \file 		AStarSearch.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-19
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef PATH_FINDER_ASTARSEARCH_H_
#define PATH_FINDER_ASTARSEARCH_H_

#include <memory>
#include <queue>
#include <deque>
#include <unordered_set>
#include "SearchAlgo.h"
#include "graph/Graph.h"
#include "Path.h"
/**
 * \class AStarQueue
 * \brief Classe concrète dérivée de priority_queue
 *
 * Classe permettant l'ajout de fonctionnalités à la classe priority_queue
 * (par exemple : clear() pour le vidage de la priority_queue)
 *
 */
template<
    class T,
    class Container = std::vector<T>,
    class Compare = std::less<typename Container::value_type>
> class AStarQueue : public std::priority_queue<T, Container, Compare>
{
public:
    explicit AStarQueue(const Compare& comp = Compare(),
                        Container&& ctnr = Container()) : std::priority_queue<T, Container, Compare>(comp, ctnr)
    {

    }
    typedef typename
        std::priority_queue<
        T,
        Container,
        Compare>::container_type::const_iterator const_iterator;

    const_iterator find(const T&val) const
    {
        auto first = this->c.cbegin();
        auto last = this->c.cend();
        while (first!=last)
        {
            if (**first==*val)
            {
                return first;
            }
            ++first;
        }
        return last;
    }

    void clear()
    {
        while(!this->empty())
        {
            this->pop();
        }
    }
};

/**
 * \class AStarSearch
 * \brief Classe concrète dérivée de SearchAlgo implémentant l'algorithme Astar
 *
 *
 */
class AStarSearch : public SearchAlgo
{
public:
    AStarSearch(const std::shared_ptr<Graph> &graph, bool reverse);
    virtual ~AStarSearch();

    virtual void search(std::shared_ptr<State> &startState, std::shared_ptr<State> &endState, Path &path) override;

    void setTieBreaking(bool tieBreaking);
protected:
    AStarQueue<std::shared_ptr<State>,
               std::deque<std::shared_ptr<State>>,
               StateComparison> m_openList;
    std::unordered_set<std::shared_ptr<State>,
                       std::hash<std::shared_ptr<State>>,
                       std::equal_to<std::shared_ptr<State>>> m_closedSet;
    bool m_tieBreaking;
};

#endif /* PATH_FINDER_ASTARSEARCH_H_ */
